import math,random
import rclpy
from rclpy.node import Node

from autoware_planning_msgs.srv import SetRoute
from geometry_msgs.msg import PoseStamped

from autoware_auto_planning_msgs.msg import PathWithLaneId,PathPointWithLaneId,Trajectory,TrajectoryPoint

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PathWithLaneId,
            '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.listener_callback,
            1)

        self.publisher_ = self.create_publisher(Trajectory, 'output', 1)
        # self.timer = self.create_timer(1.0, self.timer_callback)

    # def publish_trajectory(self):


    def listener_callback(self, msg:PathWithLaneId):
        trajectory = Trajectory()
        trajectory.header=msg.header
        for data in msg.points:
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose = data.point.pose
            trajectory_point.longitudinal_velocity_mps = data.point.longitudinal_velocity_mps
            trajectory.points.append(trajectory_point)
        self.get_logger().info('trajectory length: "%s"' % len(trajectory.points))

        # trajectory.header.stamp=self.get_clock().now().to_msg()
        self.publisher_.publish(trajectory)

class SetRouteClientAsync(Node):
    def __init__(self):
        super().__init__("set_route_client_async")
        self.get_logger().info("started")
        self.sub = self.create_subscription(PoseStamped, "/goal_pose", self.callback, 1)
        self.sub  # prevent unused variable warning
        self.cli = self.create_client(SetRoute, "/planning/mission_planning/set_route")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def callback(self, msg):
        self.get_logger().info("I heard: [%s]" % msg)
        self.send_request(msg)

    def send_request(self, goal: PoseStamped):
        request = SetRoute.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "base_link"
        # request.segments
        request.goal = goal.pose

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.future_callback)
    
    def future_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))
        else:
            self.get_logger().info("Result: %s" % (response))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    setRouteClientAsync = SetRouteClientAsync()
    rclpy.spin(setRouteClientAsync)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    setRouteClientAsync.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()