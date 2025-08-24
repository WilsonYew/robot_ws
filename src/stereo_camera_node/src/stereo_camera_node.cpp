#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode()
        : Node("stereo_camera_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("rectify_yaml", "src/stereo_camera_node/config/stereo_rectify_maps.yml");
        this->declare_parameter<std::string>("device", "/dev/video0");
        this->declare_parameter<int>("fps", 15);
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<std::string>("encoding", "mono8"); // can be "mono8" or "bgr8"
        this->declare_parameter<std::string>("left_frame_id", "left_camera");
        this->declare_parameter<std::string>("right_frame_id", "right_camera");
        this->declare_parameter<std::string>("left_topic", "/left/image_rect");
        this->declare_parameter<std::string>("right_topic", "/right/image_rect");
        this->declare_parameter<std::string>("left_info_topic", "/left/camera_info");
        this->declare_parameter<std::string>("right_info_topic", "/right/camera_info");
        this->declare_parameter<bool>("use_rectification", true);

        // Load parameters
        this->get_parameter("rectify_yaml", rectify_yaml_);
        this->get_parameter("device", device_);
        this->get_parameter("fps", fps_);
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("encoding", encoding_);
        this->get_parameter("left_frame_id", left_frame_id_);
        this->get_parameter("right_frame_id", right_frame_id_);
        this->get_parameter("left_topic", left_topic_);
        this->get_parameter("right_topic", right_topic_);
        this->get_parameter("left_info_topic", left_info_topic_);
        this->get_parameter("right_info_topic", right_info_topic_);
        this->get_parameter("use_rectification", use_rectification_);

        // Publishers
        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(left_topic_, 10);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(right_topic_, 10);
        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic_, 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic_, 10);

        // Load rectify maps if enabled
        if (use_rectification_ && !loadRectifyMaps(rectify_yaml_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load rectification YAML.");
            rclcpp::shutdown();
            return;
        }

        // Open camera
        cap_.open(device_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stereo camera: %s", device_.c_str());
            rclcpp::shutdown();
            return;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);

        // Timer loop
        int period_ms = 1000 / fps_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&StereoCameraNode::publishImages, this)
        );
    }

private:
    std::string rectify_yaml_, device_, encoding_;
    std::string left_frame_id_, right_frame_id_;
    std::string left_topic_, right_topic_;
    std::string left_info_topic_, right_info_topic_;
    bool use_rectification_;
    int fps_, width_, height_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_, right_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture cap_;
    cv::Mat left_map1_, left_map2_, right_map1_, right_map2_;
    sensor_msgs::msg::CameraInfo camera_info_left_, camera_info_right_;

    bool loadRectifyMaps(const std::string &yaml_file)
    {
        cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        // Camera parameters
        cv::Mat K_l, K_r, R_l, R_r, P_l, P_r;
        fs["K_l"] >> K_l; fs["K_r"] >> K_r;
        fs["R_l"] >> R_l; fs["R_r"] >> R_r;
        fs["P_l"] >> P_l; fs["P_r"] >> P_r;

        std::vector<double> D_l, D_r;
        fs["D_l"] >> D_l; fs["D_r"] >> D_r;

        camera_info_left_.k.fill(0); camera_info_right_.k.fill(0);
        camera_info_left_.r.fill(0); camera_info_right_.r.fill(0);
        camera_info_left_.p.fill(0); camera_info_right_.p.fill(0);

        for(int i=0;i<9;i++){
            camera_info_left_.k[i] = K_l.at<double>(i/3,i%3);
            camera_info_right_.k[i] = K_r.at<double>(i/3,i%3);
            camera_info_left_.r[i] = R_l.at<double>(i/3,i%3);
            camera_info_right_.r[i] = R_r.at<double>(i/3,i%3);
        }
        for(int i=0;i<12;i++){
            camera_info_left_.p[i] = P_l.at<double>(i/4,i%4);
            camera_info_right_.p[i] = P_r.at<double>(i/4,i%4);
        }

        camera_info_left_.d = D_l;
        camera_info_right_.d = D_r;

        int width = (int)fs["image_width"];
        int height = (int)fs["image_height"];
        camera_info_left_.width = width;
        camera_info_left_.height = height;
        camera_info_right_.width = width;
        camera_info_right_.height = height;

        // Rectify maps
        fs["map1_l"] >> left_map1_;
        fs["map2_l"] >> left_map2_;
        fs["map1_r"] >> right_map1_;
        fs["map2_r"] >> right_map2_;

        return true;
    }

    void publishImages()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured.");
            return;
        }

        // Split stereo (left = left half, right = right half)
        cv::Rect roi_left(0, 0, width_ / 2, height_);
        cv::Rect roi_right(width_ / 2, 0, width_ / 2, height_);
        cv::Mat frame_left = frame(roi_left);
        cv::Mat frame_right = frame(roi_right);

        // Rectify if enabled
        cv::Mat left_final, right_final;
        if (use_rectification_) {
            cv::remap(frame_left, left_final, left_map1_, left_map2_, cv::INTER_LINEAR);
            cv::remap(frame_right, right_final, right_map1_, right_map2_, cv::INTER_LINEAR);
        } else {
            left_final = frame_left;
            right_final = frame_right;
        }

        // Convert to grayscale if "mono8"
        if (encoding_ == "mono8") {
            cv::cvtColor(left_final, left_final, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_final, right_final, cv::COLOR_BGR2GRAY);
        }

        // Convert to ROS messages
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_, left_final).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_, right_final).toImageMsg();

        left_msg->header.stamp = this->now();
        right_msg->header.stamp = this->now();
        left_msg->header.frame_id = left_frame_id_;
        right_msg->header.frame_id = right_frame_id_;

        left_pub_->publish(*left_msg);
        right_pub_->publish(*right_msg);

        // CameraInfo
        camera_info_left_.header = left_msg->header;
        camera_info_right_.header = right_msg->header;
        left_info_pub_->publish(camera_info_left_);
        right_info_pub_->publish(camera_info_right_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
