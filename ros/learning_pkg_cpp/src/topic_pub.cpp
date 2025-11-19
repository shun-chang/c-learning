#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;  
class PubNode : public rclcpp::Node
{
public:
    PubNode() : Node("topic_pub_cpp")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PubNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "C++ 发布者节点已启动！");
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "苌得顺2025302422";  
        RCLCPP_INFO(this->get_logger(), "发布: %s", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubNode>());
    rclcpp::shutdown();
    return 0;
}