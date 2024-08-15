import math,random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from autoware_planning_msgs.srv import SetRoute
from geometry_msgs.msg import PoseStamped

from autoware_auto_planning_msgs.msg import PathWithLaneId,PathPoint,Trajectory,TrajectoryPoint
from geometry_msgs.msg import Point

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PathWithLaneId,
            '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.listener_callback,
            1)
        self.objects_subscription = self.create_subscription(
            Float64MultiArray,
            '/aichallenge/objects',
            self.objects_callback,
            1)
        self.objects=[]
        self.left_bound = []
        self.right_bound = []
        self.first_round_flag = 0

        self.publisher_ = self.create_publisher(Trajectory, 'output', 1)
        # self.timer = self.create_timer(1.0, self.timer_callback)

    # def publish_trajectory(self):
    def objects_callback(self, msg:Float64MultiArray):
        self.objects=msg.data    
    
    # def check_object_is_on_right(self, x, y):
    #     distance_min_right = 100
    #     distance_min_left = 100
    #     for data in self.right_bound:
    #         dist = (x - data.x)**2 + (y - data.y)**2
    #         if(dist<distance_min_right):
    #             distance_min_right =dist
    #     for data in self.left_bound:
    #         dist = (x - data.x)**2 + (y - data.y)**2
    #         if(dist<distance_min_left):
    #             distance_min_left =dist
    #     self.get_logger().info("Right: %s" % (distance_min_right))
    #     self.get_logger().info("Left: %s" % (distance_min_left))
    #     self.get_logger().info("Compare Result: %s" % (distance_min_right < distance_min_left))
    #     return (distance_min_right < distance_min_left)
    def find_nearest_point_in_boundary(self,trajectory_point: PathPoint,boundary, gain):
        distance_min = 100
        point_min = 1
        for i,data in enumerate(boundary):
            dist = (trajectory_point.pose.position.x - data.x)**2 + (trajectory_point.pose.position.y - data.y)**2
            if(dist<distance_min):
                distance_min =dist
                point_min = i
        trajectory_point.pose.position.x -= (trajectory_point.pose.position.x - boundary[point_min].x)/(1.9+gain)
        trajectory_point.pose.position.y -= (trajectory_point.pose.position.y - boundary[point_min].y)/(1.9+gain)
                

    def listener_callback(self, msg:PathWithLaneId):
        # self.get_logger().info(str(len(msg.points)))
        # self.get_logger().info(str(len(msg.right_bound)))
        trajectory = Trajectory()
        trajectory.header=msg.header

        # get boundary at start
        if(len(self.left_bound) == 0):
            self.left_bound = msg.left_bound # part of course ending is lost, fix me
            self.get_logger().info("left_bount_len: %s" % len(msg.left_bound))
        if(len(self.right_bound) == 0):
            self.right_bound = msg.right_bound  # part of course ending is lost, fix me
            self.get_logger().info("right_bount_len: %s" % len(msg.right_bound))
        # append boundary of goal -> start point, which is not showed at firts time , may be need fix
        if( (len(msg.left_bound) == 2) & (self.first_round_flag == 0 ) ):
            self.first_round_flag = 1
        if( (len(msg.left_bound) != 2) & (self.first_round_flag == 1 ) ):
            self.first_round_flag = 2
            self.left_bound += msg.left_bound[2:15]
            self.right_bound += msg.right_bound[2:15]
            self.get_logger().info("route merged")
        
        # transfer PathWithLaneId() to TrajectoryPoint()
        for data in msg.points:
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose = data.point.pose
            trajectory_point.longitudinal_velocity_mps = data.point.longitudinal_velocity_mps
            trajectory.points.append(trajectory_point)

        # load objects and update TrajectoryPoint() to avoid
        for i in range(round(len(self.objects)/4)):
            object_x = self.objects[i*4]
            object_y = self.objects[i*4+1]
            # object_r = self.objects[i*4+3] + 1.8

            # find nearest point in TrajectoryPoint() of object
            distance_min = 100
            point_min = -1
            for i,point in enumerate(trajectory.points):                
                dist = (point.pose.position.x - object_x)**2 + (point.pose.position.y - object_y)**2
                if(dist<distance_min):
                    distance_min =dist
                    point_min = i
            if(distance_min > 8.0):
                continue
            if(point_min > (len(trajectory.points)-2)):
                point_min = len(trajectory.points)-2

            # check object is at right or left of TrajectoryPoint()
            if(point_min < 2):
                point_min = 2 # fix out of range problem
            # if(point_min > (len(trajectory.points)-3)):
            #     point_min = len(trajectory.points)-3 # fix out of range problem
            vx1 = trajectory.points[point_min].pose.position.x - trajectory.points[point_min-2].pose.position.x
            vy1 = trajectory.points[point_min].pose.position.y - trajectory.points[point_min-2].pose.position.y
            vx2 = object_x - trajectory.points[point_min-2].pose.position.x
            vy2 = object_y - trajectory.points[point_min-2].pose.position.y  
            ans = vx1 * vy2 - vy1 * vx2 

            # <0 is right, then aviod object by driving at left side
            start_avoid = -7
            end_avoid = 6
            if( (start_avoid+point_min) < 0):
                start_avoid = 0 - point_min # fix out of range problem
            if( (end_avoid+point_min+1) > len(trajectory.points)):
                end_avoid = len(trajectory.points) - point_min - 1 # fix out of range problem

            if (ans < 0 ):
                for i in range(start_avoid,end_avoid):
                    self.find_nearest_point_in_boundary(trajectory.points[point_min+i],self.left_bound, abs(i)*0.5)
            else:
                for i in range(start_avoid,end_avoid):
                    self.find_nearest_point_in_boundary(trajectory.points[point_min+i],self.right_bound, abs(i)*0.5)

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