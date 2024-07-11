#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber()
        : Node("image_subscriber") {
        sub = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::imshow("Image Viewer", cv_ptr->image);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
