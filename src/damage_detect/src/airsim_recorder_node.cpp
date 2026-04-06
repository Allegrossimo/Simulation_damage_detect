#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <iomanip>

using namespace boost::asio;

class AirsimRecorder
{
public:
    AirsimRecorder(ros::NodeHandle& nh) : nh_(nh), it_(nh), frame_counter_(0), saved_count_(0), detected_count_(0)
    {
        // 获取参数
        nh_.param<std::string>("output_path", output_path_, "/tmp/damage_detect");
        nh_.param<std::string>("vehicle_name", vehicle_name_, "uav1");
        nh_.param<std::string>("windows_ip", windows_ip_, "10.193.133.230");
        nh_.param<int>("windows_port", windows_port_, 8891);
        nh_.param<bool>("display_enabled", display_enabled_, true);
        
        // 创建输出目录
        system(("mkdir -p " + output_path_).c_str());
        
        // 清空旧的位姿文件
        std::string pose_filename = output_path_ + "/airsim_rec.txt";
        std::ofstream clear_file(pose_filename.c_str(), std::ios::out);
        clear_file << "VehicleName\tTimeStamp\tPOS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z\tImageFile\n";
        clear_file.close();
        
        // 初始化UDP socket
        try {
            udp_socket_ = new ip::udp::socket(io_service_);
            udp_socket_->open(ip::udp::v4());
            ROS_INFO("UDP socket created for %s:%d", windows_ip_.c_str(), windows_port_);
        } catch (std::exception& e) {
            ROS_ERROR("Failed to create UDP socket: %s", e.what());
            udp_socket_ = nullptr;
        }
        
        // 订阅话题
        image_sub_ = nh_.subscribe("/airsim_node/uav1/front_left/Scene", 1, 
                                    &AirsimRecorder::imageCallback, this);
        odom_sub_ = nh_.subscribe("/uav1/mavros/local_position/odom", 1,
                                   &AirsimRecorder::odomCallback, this);
        
        // 创建显示窗口
        if (display_enabled_) {
            cv::namedWindow("Defect Detection", cv::WINDOW_NORMAL);
            cv::resizeWindow("Defect Detection", 800, 600);
        }
        
        ROS_INFO("========================================");
        ROS_INFO("AirsimRecorder started - 30fps capture mode");
        ROS_INFO("Output path: %s", output_path_.c_str());
        ROS_INFO("Pose file: %s", pose_filename.c_str());
        ROS_INFO("UDP target: %s:%d", windows_ip_.c_str(), windows_port_);
        ROS_INFO("Display enabled: %s", display_enabled_ ? "Yes" : "No");
        ROS_INFO("========================================");
    }
    
    ~AirsimRecorder()
    {
        // 清理临时文件
        for (auto& file : temp_files_) {
            remove(file.c_str());
        }
        
        if (udp_socket_) {
            udp_socket_->close();
            delete udp_socket_;
        }
        
        if (display_enabled_) {
            cv::destroyWindow("Defect Detection");
        }
        
        ROS_INFO("AirsimRecorder shutdown, saved %d images", saved_count_);
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            // 转换图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat current_image = cv_ptr->image.clone();
            
            // 获取当前时间戳
            uint64_t timestamp_ns = msg->header.stamp.toNSec();
            
            // 存储到缓冲队列
            std::lock_guard<std::mutex> lock(queue_mutex_);
            image_queue_.push({current_image, timestamp_ns});
            
            // 限制队列大小
            while (image_queue_.size() > 100) {
                image_queue_.pop();
            }
            
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (!image_queue_.empty()) {
            auto& front = image_queue_.front();
            saveData(front.first, front.second, msg);
            image_queue_.pop();
        }
    }
    
    void saveData(const cv::Mat& image, uint64_t timestamp_ns, const nav_msgs::OdometryConstPtr& odom)
    {
        frame_counter_++;
        
        // 修改：图片名称只使用时间戳，便于Python节点匹配
        std::stringstream img_filename;
        img_filename << output_path_ << "/img_" << vehicle_name_ << "_" << timestamp_ns << ".png";
        std::string img_path = img_filename.str();
        
        // 保存图片
        bool saved = cv::imwrite(img_path, image);
        if (!saved) {
            ROS_ERROR("Failed to save image: %s", img_path.c_str());
            return;
        }
        
        // 记录临时文件路径
        temp_files_.push_back(img_path);
        
        // 提取位姿信息
        double pos_x = odom->pose.pose.position.x;
        double pos_y = odom->pose.pose.position.y;
        double pos_z = odom->pose.pose.position.z;
        double q_w = odom->pose.pose.orientation.w;
        double q_x = odom->pose.pose.orientation.x;
        double q_y = odom->pose.pose.orientation.y;
        double q_z = odom->pose.pose.orientation.z;
        
        // 构建位姿记录行
        std::stringstream pose_line;
        pose_line << vehicle_name_ << "\t"
                  << timestamp_ns << "\t"
                  << std::fixed << std::setprecision(6) << pos_x << "\t"
                  << pos_y << "\t"
                  << pos_z << "\t"
                  << q_w << "\t"
                  << q_x << "\t"
                  << q_y << "\t"
                  << q_z << "\t"
                  << "img_" << vehicle_name_ << "_" << timestamp_ns << ".png\n";
        
        // 实时写入位姿文件（供Python节点读取）
        std::ofstream pose_file(output_path_ + "/airsim_rec.txt", std::ios::app);
        if (pose_file.is_open()) {
            pose_file << pose_line.str();
            pose_file.close();
        }
        
        // 显示图像
        if (display_enabled_) {
            cv::Mat display_img = image.clone();
            cv::putText(display_img, "Frame: " + std::to_string(frame_counter_),
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(display_img, "Status: Capturing",
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(display_img, "FPS: 30",
                       cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Defect Detection", display_img);
            cv::waitKey(1);
        }
        
        saved_count_++;
        
        // 每30帧打印一次统计
        if (frame_counter_ % 30 == 0) {
            ROS_INFO("Captured %d frames, saved %d images, pose file size: %ld bytes", 
                     frame_counter_, saved_count_, 
                     getFileSize(output_path_ + "/airsim_rec.txt"));
        }
        
        // 每100帧清理一次旧的临时文件（只保留最近100帧）
        if (temp_files_.size() > 100) {
            for (size_t i = 0; i < temp_files_.size() - 100; i++) {
                remove(temp_files_[i].c_str());
            }
            temp_files_.erase(temp_files_.begin(), temp_files_.begin() + (temp_files_.size() - 100));
        }
    }
    
    long getFileSize(const std::string& filename) {
        struct stat stat_buf;
        if (stat(filename.c_str(), &stat_buf) == 0) {
            return stat_buf.st_size;
        }
        return 0;
    }
    
    void sendDamageResult(const std::string& damage_msg)
    {
        if (!udp_socket_) return;
        
        try {
            ip::udp::endpoint endpoint(ip::address::from_string(windows_ip_), windows_port_);
            udp_socket_->send_to(boost::asio::buffer(damage_msg), endpoint);
            detected_count_++;
            ROS_INFO("Sent damage result #%d: %s", detected_count_, damage_msg.c_str());
        } catch (std::exception& e) {
            ROS_ERROR("Failed to send UDP message: %s", e.what());
        }
    }
    
    void updateDisplayWithDetection(const cv::Mat& image, const std::vector<cv::Rect>& detections,
                                    const std::vector<int>& types, const std::vector<float>& confs)
    {
        if (!display_enabled_) return;
        
        cv::Mat display_img = image.clone();
        
        // 绘制检测框
        cv::Scalar colors[] = {
            cv::Scalar(0, 0, 255),   // 红色 - 横向裂缝 (type 0)
            cv::Scalar(255, 0, 0),   // 蓝色 - 纵向裂缝 (type 1)
            cv::Scalar(0, 255, 255), // 黄色 - 龟裂 (type 2)
            cv::Scalar(0, 255, 0),   // 绿色 - 坑槽 (type 3)
            cv::Scalar(255, 0, 255)  // 紫色 - 裂缝 (type 4)
        };
        
        for (size_t i = 0; i < detections.size(); i++) {
            cv::rectangle(display_img, detections[i], colors[types[i] % 5], 2);
            std::string label = "Type:" + std::to_string(types[i]) + " " + 
                               std::to_string(confs[i]).substr(0, 4);
            cv::putText(display_img, label, 
                       cv::Point(detections[i].x, detections[i].y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[types[i] % 5], 1);
        }
        
        cv::putText(display_img, "Frame: " + std::to_string(frame_counter_),
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(display_img, "Status: Detecting",
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        
        cv::imshow("Defect Detection", display_img);
        cv::waitKey(1);
    }
    
    void clearTempFiles()
    {
        for (auto& file : temp_files_) {
            remove(file.c_str());
        }
        temp_files_.clear();
        ROS_DEBUG("Cleared all temporary files");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;
    ros::Subscriber odom_sub_;
    image_transport::Publisher debug_pub_;
    
    std::queue<std::pair<cv::Mat, uint64_t>> image_queue_;
    std::mutex queue_mutex_;
    
    std::string output_path_;
    std::string vehicle_name_;
    std::vector<std::string> temp_files_;
    
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket* udp_socket_;
    std::string windows_ip_;
    int windows_port_;
    
    int frame_counter_;
    bool display_enabled_;
    int saved_count_;
    int detected_count_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "airsim_recorder");
    ros::NodeHandle nh("~");
    
    AirsimRecorder recorder(nh);
    
    ros::spin();
    
    return 0;
}
