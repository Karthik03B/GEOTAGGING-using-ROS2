#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>

class Buffer : public rclcpp::Node {
public:
    Buffer()
        : Node("buffer"), count(0), trigger_count(0), trigger_active(false) {
        buffer_sub = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&Buffer::image_Callback, this, std::placeholders::_1)
        );

        trigger_sub = create_subscription<std_msgs::msg::Int32>(
            "trigger_pub", 10, std::bind(&Buffer::trigger_Callback, this, std::placeholders::_1)
        );

        last_trigger_time = std::chrono::steady_clock::now();
    }

private:
    void image_Callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        } 
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        cv::Mat frame = cv::Mat(800, 580, CV_8UC3);
        cv::cvtColor(image, frame, cv::COLOR_BGR2BGRA);

        // Check if the trigger is active and add "SAVED" text
        auto now = std::chrono::steady_clock::now();
        if (trigger_active && std::chrono::duration_cast<std::chrono::seconds>(now - last_trigger_time).count() < 1) {
            cv::putText(frame, "SAVED", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        } else {
            trigger_active = false; // Reset trigger
        }

        // Add trigger count text
        std::string trigger_count_text = "Trigger Count: " + std::to_string(trigger_count);
        cv::putText(frame, trigger_count_text, cv::Point(240, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 100, 255), 2);

        int fb = open("/dev/fb0", O_RDWR);
        if (fb < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open framebuffer device");
            return;
        } else {
            write(fb, frame.data, 800*580*3);
            close(fb);
        }
    }

    void trigger_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 1) {
            trigger_count++;
            trigger_active = true;
            last_trigger_time = std::chrono::steady_clock::now();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr buffer_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_sub;
    int count;
    int trigger_count;
    bool trigger_active;
    std::chrono::steady_clock::time_point last_trigger_time;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Buffer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}