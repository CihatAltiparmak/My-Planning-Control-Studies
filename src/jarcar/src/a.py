import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np

kalman_topic = "/jarcar/kalman_filtered_odom"

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Path, '/control/lqr/new_trajectory', 10)
        self.kalman_pub = self.create_publisher(PoseStamped, kalman_topic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.is_started = False

    def timer_callback(self):
        if not self.is_started:

            path_list = [(0.0, 0.0), (50.0, 100.0), (60.0, 200.0), (100.0, 200.0), (140.0, 100.0)] # [(0.0, 0.0), (50.0, 100.0), (60.0, 200.0), (70.0, -100.0)]

            msg = Path()
            point = PoseStamped()
            # point.pose.position.x = 0.0
            # point.pose.position.y = 0.0
            # msg.poses.append(point)
            for p in path_list:
                point = PoseStamped()
                point.pose.position.x = p[0]
                point.pose.position.y = p[1]
                msg.poses.append(point)
                self.kalman_pub.publish(point)

            self.publisher_.publish(msg)
            self.is_started = True


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
