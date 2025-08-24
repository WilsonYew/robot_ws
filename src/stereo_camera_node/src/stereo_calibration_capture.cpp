#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class StereoCalibrationCapture : public rclcpp::Node
{
public:
    StereoCalibrationCapture() : Node("stereo_calibration_capture"), frame_count_(0)
    {
        // Parameters
        this->declare_parameter<std::string>("device", "/dev/video0");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<int>("fps", 15);
        this->declare_parameter<int>("num_frames", 30);

        device_ = this->get_parameter("device").as_string();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        num_frames_ = this->get_parameter("num_frames").as_int();

        // Open camera
        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_.c_str());
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G')); // MJPEG for Pi

        // Timer for capture
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps_),
            std::bind(&StereoCalibrationCapture::captureFrame, this));
    }

private:
    void captureFrame()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        int width_half = frame.cols / 2;
        cv::Mat left = frame(cv::Rect(0, 0, width_half, frame.rows));
        cv::Mat right = frame(cv::Rect(width_half, 0, width_half, frame.rows));

        cv::Mat left_mono, right_mono;
        cv::cvtColor(left, left_mono, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right, right_mono, cv::COLOR_BGR2GRAY);

        // Save images
        std::string left_name = "left_" + std::to_string(frame_count_) + ".png";
        std::string right_name = "right_" + std::to_string(frame_count_) + ".png";
        cv::imwrite(left_name, left_mono);
        cv::imwrite(right_name, right_mono);

        RCLCPP_INFO(this->get_logger(), "Saved frame %d: %s & %s",
                    frame_count_, left_name.c_str(), right_name.c_str());

        frame_count_++;
        if (frame_count_ >= num_frames_) {
            RCLCPP_INFO(this->get_logger(), "Captured all %d frames. Shutting down.", num_frames_);
            rclcpp::shutdown();
        }
    }

    std::string device_;
    int width_, height_, fps_, num_frames_;
    int frame_count_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCalibrationCapture>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
