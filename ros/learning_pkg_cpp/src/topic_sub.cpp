#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubNode : public rclcpp::Node
{
public:
    SubNode() : Node("topic_sub_cpp")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            std::bind(&SubNode::sub_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "C++ 订阅者节点已启动！等待消息...");
    }

private:
    void sub_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "收到: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubNode>());
    rclcpp::shutdown();
    return 0;
}