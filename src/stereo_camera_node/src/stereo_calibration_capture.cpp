#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>

class StereoCalibrationCapture : public rclcpp::Node
{
public:
  StereoCalibrationCapture() : Node("stereo_calibration_capture"), frame_count_(0)
  {
    // Parameters
    this->declare_parameter<std::string>("device", "/dev/video0");
    this->declare_parameter<int>("width", 1280);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<int>("fps", 2);
    this->declare_parameter<int>("num_frames", 100);
    this->declare_parameter<int>("board_cols", 9);
    this->declare_parameter<int>("board_rows", 7);
    this->declare_parameter<bool>("require_corners", true);

    device_          = this->get_parameter("device").as_string();
    target_width_    = this->get_parameter("width").as_int();
    target_height_   = this->get_parameter("height").as_int();
    fps_             = this->get_parameter("fps").as_int();
    num_frames_      = this->get_parameter("num_frames").as_int();
    board_cols_      = this->get_parameter("board_cols").as_int();
    board_rows_      = this->get_parameter("board_rows").as_int();
    require_corners_ = this->get_parameter("require_corners").as_bool();

    // Open camera
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open device: %s", device_.c_str());
      rclcpp::shutdown();
      return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  target_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, target_height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    board_size_ = cv::Size(board_cols_, board_rows_);

    // Timer for capture
    int period_ms = std::max(1, 1000 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&StereoCalibrationCapture::captureFrame, this));
  }

private:
  void captureFrame()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Empty frame captured.");
      return;
    }

    const int W = frame.cols;
    const int H = frame.rows;
    const int halfW = W / 2;

    cv::Mat left  = frame(cv::Rect(0,      0, halfW, H));
    cv::Mat right = frame(cv::Rect(halfW,  0, halfW, H));

    cv::Mat left_mono, right_mono;
    cv::cvtColor(left,  left_mono,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, right_mono, cv::COLOR_BGR2GRAY);

    bool save_pair = true;
    if (require_corners_) {
      std::vector<cv::Point2f> cornersL, cornersR;
      bool okL = cv::findChessboardCorners(left_mono,  board_size_, cornersL);
      bool okR = cv::findChessboardCorners(right_mono, board_size_, cornersR);
      save_pair = (okL && okR);
    }

    if (!save_pair) {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1500,
                           "Corners not found in both views — skipping save.");
      return;
    }

    // Zero-padded filenames
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << frame_count_;
    std::string idx = oss.str();

    std::string left_name  = "left_"  + idx + ".png";
    std::string right_name = "right_" + idx + ".png";

    std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 3};
    cv::imwrite(left_name,  left_mono,  png_params);
    cv::imwrite(right_name, right_mono, png_params);

    RCLCPP_INFO(get_logger(), "Saved %s & %s", left_name.c_str(), right_name.c_str());

    frame_count_++;
    if (frame_count_ >= num_frames_) {
      RCLCPP_INFO(get_logger(), "Captured %d valid pairs. Shutting down.", num_frames_);
      rclcpp::shutdown();
    }
  }

  std::string device_;
  int target_width_, target_height_, fps_, num_frames_;
  int board_cols_, board_rows_;
  bool require_corners_;
  int frame_count_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Size board_size_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCalibrationCapture>());
  rclcpp::shutdown();
  return 0;
}
