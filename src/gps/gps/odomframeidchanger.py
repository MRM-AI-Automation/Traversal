import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomFrameChanger(Node):
    def __init__(self):
        super().__init__('odom_frame_changer')
        
        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_bad',   # Input topic
            self.odom_callback,
            10
        )
        
        # Publisher to republish the modified odometry message
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

        # Desired new frame ID
        self.new_child_frame_id = 'zed_imu_link'  # Set your new frame here

    def odom_callback(self, msg):
        # Modify the frame_id in the header
        msg.child_frame_id = self.new_child_frame_id
        msg.header.stamp=self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameChanger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
