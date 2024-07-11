#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class TeleopTrigger {
public:
    TeleopTrigger() : trigger_value_(0) {
        node_ = rclcpp::Node::make_shared("teleop_trigger");
        pub_ = node_->create_publisher<std_msgs::msg::Int32>("trigger/value", 10);

        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
    }

    ~TeleopTrigger() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

    void keyLoop() {
        char c;
        if (read(STDIN_FILENO, &c, 1) < 0) {
            perror("read");
            exit(-1);
        }

        if (c == 'i') {
            trigger_value_ = 1;
        } else {
            trigger_value_ = 0;
        }
    }

    void publish() {
        std_msgs::msg::Int32 msg;
        msg.data = trigger_value_;
        pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr getNode() {
        return node_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    struct termios oldt_;
    struct termios newt_;
    int trigger_value_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    TeleopTrigger teleop_trigger;
    while (rclcpp::ok()) {
        teleop_trigger.keyLoop();
        teleop_trigger.publish();
        rclcpp::spin_some(teleop_trigger.getNode());
    }
    rclcpp::shutdown();
    return 0;
}
