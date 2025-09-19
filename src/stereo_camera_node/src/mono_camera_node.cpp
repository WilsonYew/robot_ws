#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

class MonoCameraNode : public rclcpp::Node
{
public:
  MonoCameraNode() : Node("mono_camera_node")
  {
    // -------- Parameters --------
    this->declare_parameter<std::string>("device", "/dev/video0");
    this->declare_parameter<int>("fps", 10);
    this->declare_parameter<int>("width", 320);
    this->declare_parameter<int>("height", 240);
    this->declare_parameter<std::string>("frame_id", "camera_link_optical");
    this->declare_parameter<std::string>("topic", "/image_rgb");
    this->declare_parameter<bool>("use_mjpeg", true);  // try MJPEG first (lighter on CPU)

    this->get_parameter("device", device_);
    this->get_parameter("fps", fps_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("topic", topic_);
    this->get_parameter("use_mjpeg", use_mjpeg_);

    // -------- Publisher (Sensor QoS) --------
    auto sensor_qos = rclcpp::SensorDataQoS().reliable();
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, sensor_qos);

    // -------- Open camera --------
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera: %s", device_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Prefer MJPEG if available; falls back to YUYV if not
    if (use_mjpeg_) {
      cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);

    // Confirm actual settings
    int act_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int act_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double act_fps = cap_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(get_logger(), "Opened %s @ %dx%d, ~%.1f FPS", device_.c_str(), act_w, act_h, act_fps);

    const int period_ms = std::max(1, static_cast<int>(1000.0 / std::max(1, fps_)));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&MonoCameraNode::tick, this)
    );
  }

private:
  void tick()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Empty frame");
      return;
    }

    // If camera didn’t give us 320x240 (some drivers clamp), resize down
    if (frame.cols != width_ || frame.rows != height_) {
      cv::resize(frame, frame, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
    }

    // Ensure color (convert from grayscale/YUYV decode results if needed)
    if (frame.channels() == 1) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }

    // Rotate 180° (fast)
    cv::Mat frame_rot;
    cv::rotate(frame, frame_rot, cv::ROTATE_180);

    // Build header
    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id = frame_id_;

    // Publish as bgr8 on /image_rgb
    auto msg = cv_bridge::CvImage(hdr, "bgr8", frame_rot).toImageMsg();
    img_pub_->publish(*msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
      "PUB %s (bgr8 %dx%d) @ ~%d FPS", topic_.c_str(), msg->width, msg->height, fps_);
  }

  // Params
  std::string device_, frame_id_, topic_;
  int fps_{15}, width_{320}, height_{240};
  bool use_mjpeg_{true};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Video
  cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoCameraNode>());
  rclcpp::shutdown();
  return 0;
}
