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
    this->declare_parameter<std::string>("device", "/dev/video0");
    this->declare_parameter<int>("fps", 10);
    this->declare_parameter<int>("width", 640);    // SxS width (L|R)
    this->declare_parameter<int>("height", 240);
    this->declare_parameter<std::string>("encoding", "mono8"); // mono8 or bgr8

    this->declare_parameter<std::string>("maps_yaml", "src/stereo_camera_node/config/stereo_rectify_maps.yml");
    this->declare_parameter<std::string>("intrinsics_yaml", ""); // optional (can be same as maps_yaml if it also stores K/P)

    // Frame IDs – use optical frames per REP-103
    this->declare_parameter<std::string>("left_frame_id",  "left_camera_link_optical");
    this->declare_parameter<std::string>("right_frame_id", "right_camera_link_optical");

    // Topics
    this->declare_parameter<std::string>("left_topic", "/left/image_rect");
    this->declare_parameter<std::string>("right_topic", "/right/image_rect");
    this->declare_parameter<std::string>("left_info_topic", "/left/camera_info");
    this->declare_parameter<std::string>("right_info_topic", "/right/camera_info");

    // Intrinsics fallback (used only if YAML doesn’t provide them)
    this->declare_parameter<double>("fx", 400.0);
    this->declare_parameter<double>("fy", 400.0);
    this->declare_parameter<double>("cx", 320.0);
    this->declare_parameter<double>("cy", 120.0);
    this->declare_parameter<double>("baseline_m", 0.020); // 20 mm

    // --- Get params ---
    this->get_parameter("device", device_);
    this->get_parameter("fps", fps_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("encoding", encoding_);
    this->get_parameter("maps_yaml", maps_yaml_);
    this->get_parameter("intrinsics_yaml", intrinsics_yaml_);
    this->get_parameter("left_frame_id", left_frame_id_);
    this->get_parameter("right_frame_id", right_frame_id_);
    this->get_parameter("left_topic", left_topic_);
    this->get_parameter("right_topic", right_topic_);
    this->get_parameter("left_info_topic", left_info_topic_);
    this->get_parameter("right_info_topic", right_info_topic_);
    this->get_parameter("fx", fx_);
    this->get_parameter("fy", fy_);
    this->get_parameter("cx", cx_);
    this->get_parameter("cy", cy_);
    this->get_parameter("baseline_m", baseline_);

    // --- Publishers (SensorData QoS) ---
    auto qos = rclcpp::SensorDataQoS();
    left_pub_       = this->create_publisher<sensor_msgs::msg::Image>(left_topic_, qos);
    right_pub_      = this->create_publisher<sensor_msgs::msg::Image>(right_topic_, qos);
    left_info_pub_  = this->create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic_, qos);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic_, qos);

    // --- Load maps ---
    if (!loadMaps(maps_yaml_)) {
      RCLCPP_ERROR(get_logger(), "Failed to read maps from: %s", maps_yaml_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Set sizes from map size (one eye)
    camera_info_left_.width  = left_map1_.cols;
    camera_info_left_.height = left_map1_.rows;
    camera_info_right_.width  = right_map1_.cols;
    camera_info_right_.height = right_map1_.rows;

    // --- Load intrinsics/projections ---
    if (!loadIntrinsicsFromYaml(intrinsics_yaml_.empty() ? maps_yaml_ : intrinsics_yaml_)) {
      // Fallback to parameters
      RCLCPP_WARN(get_logger(), "No intrinsics found in YAML; using fx/fy/cx/cy/baseline params.");
      fillRectifiedCameraInfoFromParams();
    }

    // --- Open camera ---
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open camera: %s", device_.c_str());
      rclcpp::shutdown();
      return;
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);

    // --- Timer ---
    int period_ms = std::max(1, 1000 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&StereoCameraNode::tick, this)
    );
  }

private:
  // Params
  std::string device_, encoding_;
  std::string maps_yaml_, intrinsics_yaml_;
  std::string left_frame_id_, right_frame_id_;
  std::string left_topic_, right_topic_, left_info_topic_, right_info_topic_;
  int fps_{15}, width_{640}, height_{240};
  double fx_{400.0}, fy_{400.0}, cx_{320.0}, cy_{120.0}, baseline_{0.020};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_, right_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Video + rectification maps
  cv::VideoCapture cap_;
  cv::Mat left_map1_, left_map2_, right_map1_, right_map2_;
  sensor_msgs::msg::CameraInfo camera_info_left_, camera_info_right_;

  static bool tryReadMat(cv::FileStorage &fs, const std::string &key, cv::Mat &dst)
  {
    if (!fs[key].empty()) { fs[key] >> dst; return !dst.empty(); }
    return false;
  }

  bool loadMaps(const std::string &yaml_path)
  {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    bool ok = true;
    ok &= tryReadMat(fs, "map1_l", left_map1_);
    ok &= tryReadMat(fs, "map2_l", left_map2_);
    ok &= tryReadMat(fs, "map1_r", right_map1_);
    ok &= tryReadMat(fs, "map2_r", right_map2_);
    fs.release();

    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Missing one or more keys: map1_l/map2_l/map1_r/map2_r");
      return false;
    }
    if (left_map1_.size() != left_map2_.size() || right_map1_.size() != right_map2_.size()) {
      RCLCPP_ERROR(get_logger(), "Map sizes mismatch between map1/map2");
      return false;
    }
    return true;
  }

  bool loadIntrinsicsFromYaml(const std::string &yaml_path)
  {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    bool got = false;

    // Try common key names written by calibration scripts
    got |= tryReadMat(fs, "K1_rect", K_l);
    got |= tryReadMat(fs, "K2_rect", K_r);
    got |= tryReadMat(fs, "P1", P_l);
    got |= tryReadMat(fs, "P2", P_r);
    if (!got) {
      // Alternate names
      tryReadMat(fs, "K_left", K_l);
      tryReadMat(fs, "K_right", K_r);
      tryReadMat(fs, "P_left", P_l);
      tryReadMat(fs, "P_right", P_r);
    }
    tryReadMat(fs, "R1", R_l);
    tryReadMat(fs, "R2", R_r);
    tryReadMat(fs, "D1_rect", D_l);
    tryReadMat(fs, "D2_rect", D_r);
    fs.release();

    // Validate
    if (P_l.empty() || P_r.empty() || P_l.cols != 4 || P_r.cols != 4) {
      RCLCPP_WARN(get_logger(), "No valid P1/P2 in YAML; will use fx/fy/cx/cy + baseline params.");
      return false;
    }

    // Extract fx,fy,cx,cy from P
    double fx = P_l.at<double>(0,0);
    double fy = P_l.at<double>(1,1);
    double cx = P_l.at<double>(0,2);
    double cy = P_l.at<double>(1,2);
    double Tx = P_r.at<double>(0,3); // should be -fx * baseline

    camera_info_left_.distortion_model = "plumb_bob";
    camera_info_left_.d.assign(5, 0.0);
    camera_info_left_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    camera_info_left_.r = {1,0,0, 0,1,0, 0,0,1};
    camera_info_left_.p = {fx, 0.0, cx, 0.0,
                           0.0, fy, cy, 0.0,
                           0.0, 0.0, 1.0, 0.0};

    camera_info_right_ = camera_info_left_;
    camera_info_right_.p[3] = Tx; // already contains -fx*B from YAML

    // Save the numbers for reference/fallback
    fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    if (fx_ != 0.0) baseline_ = -Tx / fx_; // derive B
    return true;
  }

  void fillRectifiedCameraInfoFromParams()
  {
    camera_info_left_.distortion_model = "plumb_bob";
    camera_info_left_.d.assign(5, 0.0);
    camera_info_left_.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
    camera_info_left_.r = {1,0,0, 0,1,0, 0,0,1};
    camera_info_left_.p = {fx_, 0.0, cx_, 0.0,
                           0.0, fy_, cy_, 0.0,
                           0.0, 0.0, 1.0, 0.0};

    camera_info_right_ = camera_info_left_;
    camera_info_right_.p[3] = -fx_ * baseline_;
  }

  void tick()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Empty frame");
      return;
    }

    const int eye_w = width_ / 2;
    const int eye_h = height_;
    if (frame.cols < width_ || frame.rows < height_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Captured frame (%dx%d) smaller than expected (%dx%d)",
                           frame.cols, frame.rows, width_, height_);
      return;
    }

    cv::Mat frame_left  = frame(cv::Rect(0,      0, eye_w, eye_h)).clone();
    cv::Mat frame_right = frame(cv::Rect(eye_w,  0, eye_w, eye_h)).clone();

    // Ensure input matches map size (best: match at source)
    if (frame_left.size() != left_map1_.size())
      cv::resize(frame_left, frame_left, left_map1_.size(), 0, 0, cv::INTER_LINEAR);
    if (frame_right.size() != right_map1_.size())
      cv::resize(frame_right, frame_right, right_map1_.size(), 0, 0, cv::INTER_LINEAR);

    // Rectify
    cv::Mat left_rect, right_rect;
    cv::remap(frame_left,  left_rect,  left_map1_,  left_map2_,  cv::INTER_LINEAR);
    cv::remap(frame_right, right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);

    // Convert encoding if needed
    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id = left_frame_id_;

    sensor_msgs::msg::Image left_msg, right_msg;
    if (encoding_ == "mono8") {
      if (left_rect.channels() == 3)  cv::cvtColor(left_rect,  left_rect,  cv::COLOR_BGR2GRAY);
      if (right_rect.channels() == 3) cv::cvtColor(right_rect, right_rect, cv::COLOR_BGR2GRAY);
      left_msg  = *cv_bridge::CvImage(hdr, "mono8", left_rect).toImageMsg();
      hdr.frame_id = right_frame_id_;
      right_msg = *cv_bridge::CvImage(hdr, "mono8", right_rect).toImageMsg();
    } else {
      // default bgr8
      left_msg  = *cv_bridge::CvImage(hdr, "bgr8", left_rect).toImageMsg();
      hdr.frame_id = right_frame_id_;
      right_msg = *cv_bridge::CvImage(hdr, "bgr8", right_rect).toImageMsg();
    }

    // Publish images
    left_pub_->publish(left_msg);
    right_pub_->publish(right_msg);

    // Publish CameraInfo (rectified)
    camera_info_left_.header  = left_msg.header;                 // stamp + left frame
    camera_info_right_.header = right_msg.header;                // stamp + right frame
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
