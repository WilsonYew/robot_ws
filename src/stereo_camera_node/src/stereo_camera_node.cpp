#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode() : Node("stereo_camera_node_pi"), running_(true)
    {
        this->declare_parameter<std::string>("device", "/dev/video0");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<int>("fps", 15);

        device_ = this->get_parameter("device").as_string();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();

        // Open camera with V4L2
        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_.c_str());
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G')); // MJPEG

        rclcpp::QoS qos(5);
        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", qos);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", qos);

        // Start capture thread
        capture_thread_ = std::thread(&StereoCameraNode::captureLoop, this);
    }

    ~StereoCameraNode() {
        running_ = false;
        if (capture_thread_.joinable())
            capture_thread_.join();
    }

private:
    void captureLoop() {
        cv::Mat frame;
        int width_half = width_ / 2;

        while (rclcpp::ok() && running_) {
            if (!cap_.read(frame) || frame.empty()) continue;

            // Split left/right (ROI, no copy)
            cv::Mat left = frame(cv::Rect(0, 0, width_half, height_));
            cv::Mat right = frame(cv::Rect(width_half, 0, width_half, height_));

            // Convert to mono8
            cv::Mat left_mono, right_mono;
            cv::cvtColor(left, left_mono, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right, right_mono, cv::COLOR_BGR2GRAY);

            auto header = std_msgs::msg::Header();
            header.stamp = this->now();
            header.frame_id = "camera_frame";

            // Publish left/right
            left_pub_->publish(*cv_bridge::CvImage(header, "mono8", left_mono).toImageMsg());
            right_pub_->publish(*cv_bridge::CvImage(header, "mono8", right_mono).toImageMsg());

            // Sleep to match FPS
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps_));
        }
    }

    std::string device_;
    int width_, height_, fps_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
    std::thread capture_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}