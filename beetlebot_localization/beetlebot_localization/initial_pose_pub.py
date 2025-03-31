import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('InitialPose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.x = 0.0
        self.y = 0.0

    def clicked_point_callback(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.get_logger().info('X point: "%s"' % msg.point.x)
        self.get_logger().info('Y point: "%s"' % msg.point.y)

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 0.0
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.pose.pose.position.x)


def main(args=None):
    rclpy.init(args=args)

    InitialPose_publisher = InitialPosePublisher()

    rclpy.spin(InitialPose_publisher)

    # Destroy the node explicitly
    InitialPose_publisher.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main()