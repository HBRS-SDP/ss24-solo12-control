import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(String, 'control_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = 'Control command for solo12 robot %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()