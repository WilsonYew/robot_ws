#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode() : Node("stereo_camera_node")
  {
    // --- Parameters ---
    this->declare_parameter<std::string>("maps_yaml", "src/stereo_camera_node/config/stereo_rectify_maps.yml");
    this->declare_parameter<std::string>("device", "/dev/video0");
    this->declare_parameter<int>("fps", 15);
    this->declare_parameter<int>("width", 640);   // side-by-side width (L|R)
    this->declare_parameter<int>("height", 240);
    this->declare_parameter<std::string>("encoding", "mono8"); // "mono8" or "bgr8"
    this->declare_parameter<std::string>("left_frame_id", "left_camera");
    this->declare_parameter<std::string>("right_frame_id", "right_camera");
    this->declare_parameter<std::string>("left_topic", "/left/image_rect");
    this->declare_parameter<std::string>("right_topic", "/right/image_rect");
    this->declare_parameter<std::string>("left_info_topic", "/left/camera_info");
    this->declare_parameter<std::string>("right_info_topic", "/right/camera_info");

    this->get_parameter("maps_yaml", maps_yaml_);
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

    // --- Publishers ---
    left_pub_  = this->create_publisher<sensor_msgs::msg::Image>(left_topic_,  10);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(right_topic_, 10);
    left_info_pub_  = this->create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic_,  10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic_, 10);

    // --- Load maps (maps-only YAML) ---
    if (!loadMaps(maps_yaml_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read maps from YAML: %s", maps_yaml_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Set CameraInfo sizes from map size (one eye)
    camera_info_left_.width  = left_map1_.cols;
    camera_info_left_.height = left_map1_.rows;
    camera_info_right_.width  = right_map1_.cols;
    camera_info_right_.height = right_map1_.rows;

    // --- Open camera ---
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", device_.c_str());
      rclcpp::shutdown();
      return;
    }
    // Try MJPG to keep USB bandwidth lower; harmless if not supported
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);   // reduce lag

    // --- Timer ---
    int period_ms = std::max(1, 1000 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&StereoCameraNode::tick, this)
    );
  }

private:
  // Params
  std::string maps_yaml_, device_, encoding_;
  std::string left_frame_id_, right_frame_id_;
  std::string left_topic_, right_topic_, left_info_topic_, right_info_topic_;
  int fps_{15}, width_{640}, height_{240};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_, right_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Video + rectification maps
  cv::VideoCapture cap_;
  cv::Mat left_map1_, left_map2_, right_map1_, right_map2_;
  sensor_msgs::msg::CameraInfo camera_info_left_, camera_info_right_;

  // Accepts maps saved via OpenCV FileStorage:
  // keys: map1_l, map2_l, map1_r, map2_r (CV_16SC2 or CV_32FC1/FC2)
  bool loadMaps(const std::string &yaml_path)
  {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open YAML: %s", yaml_path.c_str());
      return false;
    }

    fs["map1_l"] >> left_map1_;
    fs["map2_l"] >> left_map2_;
    fs["map1_r"] >> right_map1_;
    fs["map2_r"] >> right_map2_;
    fs.release();

    if (left_map1_.empty() || left_map2_.empty() || right_map1_.empty() || right_map2_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "One or more rectification maps are missing/empty.");
      return false;
    }
    // Basic sanity: sizes must match within each eye
    if (left_map1_.size() != left_map2_.size() || right_map1_.size() != right_map2_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Map sizes mismatch between map1/map2.");
      return false;
    }
    return true;
  }

  void tick()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty frame");
      return;
    }

    // Split side-by-side into L/R
    const int eye_w = width_ / 2;
    const int eye_h = height_;
    if (frame.cols < width_ || frame.rows < height_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Captured frame (%dx%d) smaller than expected (%dx%d)",
                           frame.cols, frame.rows, width_, height_);
      return;
    }
    cv::Mat frame_left  = frame(cv::Rect(0,          0, eye_w, eye_h));
    cv::Mat frame_right = frame(cv::Rect(eye_w, 0, eye_w, eye_h));

    // Ensure input matches map size; resize if needed (best to fix at source)
    if (frame_left.size() != left_map1_.size())
      cv::resize(frame_left, frame_left, left_map1_.size(), 0, 0, cv::INTER_LINEAR);
    if (frame_right.size() != right_map1_.size())
      cv::resize(frame_right, frame_right, right_map1_.size(), 0, 0, cv::INTER_LINEAR);

    // Rectify via precomputed maps
    cv::Mat left_rect, right_rect;
    cv::remap(frame_left,  left_rect,  left_map1_,  left_map2_,  cv::INTER_LINEAR);
    cv::remap(frame_right, right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);

    // Convert to mono if requested
    if (encoding_ == "mono8") {
      cv::cvtColor(left_rect,  left_rect,  cv::COLOR_BGR2GRAY);
      cv::cvtColor(right_rect, right_rect, cv::COLOR_BGR2GRAY);
    } else if (encoding_ != "bgr8") {
      // Fallback: enforce bgr8 if unknown encoding
      encoding_ = "bgr8";
    }

    // ROS messages
    auto stamp = this->now();
    auto left_msg  = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_, left_rect).toImageMsg();
    auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_, right_rect).toImageMsg();
    left_msg->header.stamp  = stamp;
    right_msg->header.stamp = stamp;
    left_msg->header.frame_id  = left_frame_id_;
    right_msg->header.frame_id = right_frame_id_;

    left_pub_->publish(*left_msg);
    right_pub_->publish(*right_msg);

    camera_info_left_.header  = left_msg->header;
    camera_info_right_.header = right_msg->header;
    left_info_pub_->publish(camera_info_left_);
    right_info_pub_->publish(camera_info_right_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCameraNode>());
  rclcpp::shutdown();
  return 0;
}
