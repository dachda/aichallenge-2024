import math,random
import rclpy
from rclpy.node import Node

from autoware_auto_planning_msgs.msg import PathPointWithLaneId,Trajectory,TrajectoryPoint

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.path=PathPointWithLaneId()
        self.subscription = self.create_subscription(
            PathPointWithLaneId,
            '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.listener_callback,
            1)

        self.publisher_ = self.create_publisher(Trajectory, '/planning/scenario_planning/trajectory', 1)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Trajectory()
        msg.header = self.path.header
        for data in self.path.points:
            trajectory_point = TrajectoryPoint()
        

    def listener_callback(self, msg:PathPointWithLaneId):
        self.path = msg
        self.get_logger().info('path: "%s"' % self.path)

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