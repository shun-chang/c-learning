#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"

class TurtleFollowNode : public rclcpp::Node
{
public:
    TurtleFollowNode() : Node("turtle_follow_node")
    {

        spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!spawn_turtle_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "等待 spawn 服务...");
        }
        auto spawn_req = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_req->x = 5.0;  
        spawn_req->y = 5.0;  
        spawn_req->theta = 0.0;
        spawn_req->name = "turtle2";
        spawn_turtle_client_->async_send_request(spawn_req);

        turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleFollowNode::turtle1_pose_callback, this, std::placeholders::_1)
        );

        turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose",
            10,
            std::bind(&TurtleFollowNode::turtle2_pose_callback, this, std::placeholders::_1)
        );

        turtle2_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleFollowNode::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "跟随节点启动成功！turtle2 跟随 turtle1");
    }

private:

    turtlesim::msg::Pose turtle1_pose_;
    turtlesim::msg::Pose turtle2_pose_;


    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_vel_pub_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void turtle1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pose_ = *msg;
    }

    void turtle2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle2_pose_ = *msg;
    }

    void control_loop()
    {
        auto vel_cmd = geometry_msgs::msg::Twist();

        double dx = turtle1_pose_.x - turtle2_pose_.x;
        double dy = turtle1_pose_.y - turtle2_pose_.y;

        double distance = sqrt(dx*dx + dy*dy);
        double target_angle = atan2(dy, dx);  

        double angle_error = target_angle - turtle2_pose_.theta;
        angle_error = atan2(sin(angle_error), cos(angle_error));

        double distance_threshold = 0.5;  
        double linear_speed_gain = 1.0;   
        double angular_speed_gain = 4.0;  

        if (distance > distance_threshold) {
            vel_cmd.linear.x = linear_speed_gain * distance;
        } else {
            vel_cmd.linear.x = 0.0;
        }

        vel_cmd.angular.z = angular_speed_gain * angle_error;

        turtle2_vel_pub_->publish(vel_cmd);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleFollowNode>());
    rclcpp::shutdown();
    return 0;
}