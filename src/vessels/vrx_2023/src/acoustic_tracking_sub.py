import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg  import ParamVec
from nav_msgs.msg import Odometry 



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('acoustic_tracking_sub')
        self.pinger_subscription = self.create_subscription(
            ros_gz_interfaces/msg/ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.pinger_callback,
            10)
        self.odometry_subscription = self.create_subscription(
            nav_msgs/msg/Odometry,
            'odometry/filtered',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def pinger_callback(self, msg):
        self.get_logger().info('Pinger heard: "%s"' % msg.params[0])
    def odom_callback(self, msg):
        self.get_logger().info('Odom heard x position: "%s"' % msg.pose.pose.position.x)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()