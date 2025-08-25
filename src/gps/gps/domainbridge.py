import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DomainBridge(Node):
    def __init__(self):
        super().__init__('domain_bridge_node')
        
        # Publisher for domain 2
        self.publisher_ = self.create_publisher(String, 'message2', 10)

        # Subscriber for domain 1
        self.subscription = self.create_subscription(
            String,
            '/message',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Forwarding message: {msg.data}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    domain_bridge = DomainBridge()

    rclpy.spin(domain_bridge)

    domain_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
