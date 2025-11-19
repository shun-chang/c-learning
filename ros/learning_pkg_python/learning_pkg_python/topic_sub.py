import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubNode(Node):
    def __init__(self):
        super().__init__('topic_sub')
        # 订阅名为'chatter'的话题，消息类型为String
        self.sub = self.create_subscription(
            String,
            'chatter',
            self.callback,  # 收到消息时执行的函数
            10  # 队列大小
        )

    def callback(self, msg):
        # 打印收到的消息内容
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    rclpy.spin(node)  # 保持节点运行，等待消息
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()