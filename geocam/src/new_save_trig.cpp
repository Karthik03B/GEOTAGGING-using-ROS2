#include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/Float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node")
    {
        // Parameters
        declare_parameter("threshold", 1400);
        get_parameter("threshold", threshold_);

        // Publishers and Subscribers
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageSaverNode::image_callback, this, std::placeholders::_1));

        navsat_subscriber_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ap/navsat/navsat0", 10,
            std::bind(&ImageSaverNode::navsat_callback, this, std::placeholders::_1));

        frequency_subscriber_ = create_subscription<std_msgs::msg::Float32>(
            "/frequency_topic", 10,
            std::bind(&ImageSaverNode::frequency_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (triggered_)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                std::string file_name = "image" + std::to_string(image_count_) + ".jpg";
                cv::imwrite(images_dir_ + "/" + file_name, cv_ptr->image);
                image_names_file_ << file_name << "," << navsat_data_.latitude << "," << navsat_data_.longitude << std::endl;
                image_count_++;
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }
    }

    void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        navsat_data_ = *msg;
    }

    void frequency_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (msg->data > threshold_)
        {
            triggered_ = true;
            // Create folder for saving images if not exists
            images_dir_ = "/home/" + std::string(getenv("USER")) + "/images_saved/images";
            system(("mkdir -p " + images_dir_).c_str());
            // Open file for saving image names and GPS data
            image_names_file_.open("/home/" + std::string(getenv("USER")) + "/images_saved/image_names.txt", std::ios_base::app);
        }
        else
        {
            triggered_ = false;
            // Close file after saving
            if (image_names_file_.is_open())
            {
                image_names_file_.close();
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr frequency_subscriber_;

    float threshold_;
    bool triggered_ = false;
    int image_count_ = 1;
    std::string images_dir_;
    sensor_msgs::msg::NavSatFix navsat_data_;
    std::ofstream image_names_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

