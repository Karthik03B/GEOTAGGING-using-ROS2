#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <vector>
 class Encoding : public rclcpp::Node{
    public:
    Encoding(): Node("subscriber"), count(0){
        sub1 = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 1, std::bind(&Encoding::image_callback,this, std::placeholders::_1));
    }
    private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
         cv_bridge::CvImagePtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        } 
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return ;
        }
        cv::Mat image = cv_ptr-> image;
        std::vector<uint8_t> bytes = imageToBytes(image);     
 
       }
         std::vector<uint8_t> imageToBytes(const cv::Mat& image) {
         std::vector<uint8_t> bytes;
         cv::imencode(".jpg", image, bytes);
         std::cout << "Image size in bytes: " <<bytes.size() << std::endl;
         return bytes;
        }

    
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1;
        int count;
 };
 int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Encoding>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


