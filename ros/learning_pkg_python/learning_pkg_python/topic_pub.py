import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubNode(Node):
    def __init__(self):
        super().__init__('topic_pub')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.create_timer(0.5, self.cb)

    def cb(self):
        msg = String()
        msg.data = '苌得顺2025302422'
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()