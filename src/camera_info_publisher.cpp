#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <iostream>
#include <string>
#include <vector>

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher() : Node("camera_info_publisher")
    {
        this->declare_parameter<std::string>("frame_id", "camera");
        this->declare_parameter<int>("image_width", 640);
        this->declare_parameter<int>("image_height", 480);
        this->declare_parameter<std::string>("camera_name", "camera");
        this->declare_parameter<std::string>("distortion_model", "plumb_bob");
        this->declare_parameter<double>("publish_rate", 1.0);
        this->declare_parameter<std::vector<double>>("camera_matrix.data", std::vector<double>(9, 0.0));
        this->declare_parameter<std::vector<double>>("distortion_coefficients.data", std::vector<double>(5, 0.0));
        this->declare_parameter<std::vector<double>>("rectification_matrix.data", std::vector<double>(9, 0.0));
        this->declare_parameter<std::vector<double>>("projection_matrix.data", std::vector<double>(12, 0.0));
        this->declare_parameter<bool>("use_image_trigger", false);
        this->declare_parameter<std::string>("image_topic", "/camera_image");
        this->declare_parameter<std::string>("image_type", "raw"); // "raw" or "compressed"
        
        use_image_trigger_ = this->get_parameter("use_image_trigger").as_bool();
        image_topic_ = this->get_parameter("image_topic").as_string();
        image_type_ = this->get_parameter("image_type").as_string();
        
        setupCameraInfo();
        
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        
        if (use_image_trigger_) {
            setupImageSubscription();
            RCLCPP_INFO(this->get_logger(), "Camera info publisher started in IMAGE TRIGGER mode");
            RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s (type: %s)", image_topic_.c_str(), image_type_.c_str());
            RCLCPP_INFO(this->get_logger(), "  Note: frame_id, width, height will be synchronized with image messages");
        } else {
            setupTimer();
            RCLCPP_INFO(this->get_logger(), "Camera info publisher started in TIMER mode");
            double publish_rate = this->get_parameter("publish_rate").as_double();
            RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate);
        }
        
        RCLCPP_INFO(this->get_logger(), "  Initial Frame ID: %s", camera_info_.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "  Initial Image size: %dx%d", camera_info_.width, camera_info_.height);
        RCLCPP_INFO(this->get_logger(), "  Distortion model: %s", camera_info_.distortion_model.c_str());
        RCLCPP_INFO(this->get_logger(), "  Topic can be remapped using: --ros-args -r camera_info:=your_topic_name");
    }

private:
    void setupCameraInfo()
    {
        camera_info_.header.frame_id = this->get_parameter("frame_id").as_string();
        camera_info_.width = static_cast<uint32_t>(this->get_parameter("image_width").as_int());
        camera_info_.height = static_cast<uint32_t>(this->get_parameter("image_height").as_int());
        camera_info_.distortion_model = this->get_parameter("distortion_model").as_string();
        
        // Camera matrix (K)
        auto camera_matrix = this->get_parameter("camera_matrix.data").as_double_array();
        if (camera_matrix.size() == 9) {
            std::copy(camera_matrix.begin(), camera_matrix.end(), camera_info_.k.begin());
            RCLCPP_INFO(this->get_logger(), "Camera matrix loaded: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                       camera_matrix[0], camera_matrix[1], camera_matrix[2],
                       camera_matrix[3], camera_matrix[4], camera_matrix[5],
                       camera_matrix[6], camera_matrix[7], camera_matrix[8]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Camera matrix must have 9 elements, got %zu", camera_matrix.size());
        }
        
        // Distortion coefficients (D)
        camera_info_.d = this->get_parameter("distortion_coefficients.data").as_double_array();
        RCLCPP_INFO(this->get_logger(), "Distortion coefficients loaded: %zu elements", camera_info_.d.size());
        
        // Rectification matrix (R)
        auto rect_matrix = this->get_parameter("rectification_matrix.data").as_double_array();
        if (rect_matrix.size() == 9) {
            std::copy(rect_matrix.begin(), rect_matrix.end(), camera_info_.r.begin());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Rectification matrix must have 9 elements, got %zu", rect_matrix.size());
        }
        
        // Projection matrix (P)
        auto proj_matrix = this->get_parameter("projection_matrix.data").as_double_array();
        if (proj_matrix.size() == 12) {
            std::copy(proj_matrix.begin(), proj_matrix.end(), camera_info_.p.begin());
            RCLCPP_INFO(this->get_logger(), "Projection matrix loaded: 12 elements");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Projection matrix must have 12 elements, got %zu", proj_matrix.size());
        }
    }
    
    void setupTimer()
    {
        double publish_rate = this->get_parameter("publish_rate").as_double();
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(timer_period, std::bind(&CameraInfoPublisher::publishCameraInfo, this));
    }
    
    void setupImageSubscription()
    {
        if (image_type_ == "compressed") {
            compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                image_topic_, 10, 
                std::bind(&CameraInfoPublisher::compressedImageCallback, this, std::placeholders::_1));
        } else {
            // Default to raw image
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic_, 10, 
                std::bind(&CameraInfoPublisher::imageCallback, this, std::placeholders::_1));
        }
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        updateCameraInfoFromImage(msg->header, msg->width, msg->height);
        publishCameraInfo();
    }
    
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        // For compressed images, we don't have direct access to width/height
        // Only update frame_id from header
        updateCameraInfoFromHeader(msg->header);
        publishCameraInfo();
    }
    
    void updateCameraInfoFromImage(const std_msgs::msg::Header& header, uint32_t width, uint32_t height)
    {
        // Update frame_id if changed
        if (!header.frame_id.empty() && camera_info_.header.frame_id != header.frame_id) {
            RCLCPP_INFO(this->get_logger(), 
                       "Frame ID updated: '%s' -> '%s'", 
                       camera_info_.header.frame_id.c_str(),
                       header.frame_id.c_str());
            camera_info_.header.frame_id = header.frame_id;
        }
        
        // Update resolution if changed
        if (camera_info_.width != width || camera_info_.height != height) {
            RCLCPP_INFO(this->get_logger(), 
                       "Image resolution updated: %dx%d -> %dx%d", 
                       camera_info_.width, camera_info_.height,
                       width, height);
            camera_info_.width = width;
            camera_info_.height = height;
        }
    }
    
    void updateCameraInfoFromHeader(const std_msgs::msg::Header& header)
    {
        // Update frame_id if changed (for compressed images)
        if (!header.frame_id.empty() && camera_info_.header.frame_id != header.frame_id) {
            RCLCPP_INFO(this->get_logger(), 
                       "Frame ID updated: '%s' -> '%s'", 
                       camera_info_.header.frame_id.c_str(),
                       header.frame_id.c_str());
            camera_info_.header.frame_id = header.frame_id;
        }
    }
    
    void publishCameraInfo()
    {
        // Set timestamp to current time (publish time)
        camera_info_.header.stamp = this->get_clock()->now();
        camera_info_pub_->publish(camera_info_);
    }
    
    // メンバ変数
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    sensor_msgs::msg::CameraInfo camera_info_;
    
    // 新機能用のメンバ変数
    bool use_image_trigger_;
    std::string image_topic_;
    std::string image_type_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CameraInfoPublisher>();
    
    try {
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_info_publisher"), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}