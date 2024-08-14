import math,random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
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
        self.objects_subscription = self.create_subscription(
            Float64MultiArray,
            '/aichallenge/objects',
            self.objects_callback,
            1)
        self.objects=[]
        self.left_bound = []
        self.right_bound = []
        self.first_round = True

        self.publisher_ = self.create_publisher(Trajectory, 'output', 1)
        # self.timer = self.create_timer(1.0, self.timer_callback)

    # def publish_trajectory(self):
    def objects_callback(self, msg:Float64MultiArray):
        self.objects=msg.data    


    def listener_callback(self, msg:PathWithLaneId):
        # self.get_logger().info(str(len(msg.points)))
        self.get_logger().info(str(len(msg.left_bound)))
        # self.get_logger().info(str(len(msg.right_bound)))
        trajectory = Trajectory()
        trajectory.header=msg.header
        if(len(self.left_bound) == 0):
            self.left_bound = msg.left_bound # part of course ending is lost, fix me
        if(len(self.right_bound) == 0):
            self.right_bound = msg.right_bound  # part of course ending is lost, fix me
        # if( (len(msg.left_bound) == 2) & self.first_round ):
        #     self.first_round = False
        #     self.left_bound += self.left_bound
        #     self.get_logger().info("route merged") 
        #     for i,data in enumerate(self.right_bound):
        #         if(round(data.x) == round(self.right_bound[0])):
        #            self.get_logger().info("Result: %s" % str(i)) 

        for data in msg.points:
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose = data.point.pose
            # trajectory_point.pose.position.x += 1
            # trajectory_point.pose.position.y += 1
            trajectory_point.longitudinal_velocity_mps = data.point.longitudinal_velocity_mps
            trajectory.points.append(trajectory_point)
        for i in range(round(len(self.objects)/4)):
            object_x = self.objects[i*4]
            object_y = self.objects[i*4+1]
            object_r = self.objects[i*4+3] + 1.3

            # find nearest point
            distance_min = 100
            point_min = -1
            for i,point in enumerate(trajectory.points):                
                dist = (point.pose.position.x - object_x)**2 + (point.pose.position.y - object_y)**2
                if(dist<distance_min):
                    distance_min =dist
                    point_min = i
            if(distance_min > object_r):
                continue
            if(point_min < 3):
                point_min = 3
            if(point_min > (len(trajectory.points)-2)):
                point_min = len(trajectory.points)-2
            # check right or left
            vx1 = trajectory.points[point_min+1].pose.position.x - trajectory.points[point_min-1].pose.position.x
            vy1 = trajectory.points[point_min+1].pose.position.y - trajectory.points[point_min-1].pose.position.y
            vx2 = object_x - trajectory.points[point_min-1].pose.position.x
            vy2 = object_y - trajectory.points[point_min-1].pose.position.y
            # <0 is right
            ans = vx1 * vy2 - vy1 * vx2
            if (ans < 0 ):
                # self.get_logger().info('right object')
                # find nearest point on left site
                left_distance_min_pre = 100
                left_distance_min_after = 100
                left_point_min_pre = 1
                left_point_min_after = 1
                for i,data in enumerate(self.left_bound):
                    dist_pre = (trajectory.points[point_min-1].pose.position.x - data.x)**2 + (trajectory.points[point_min-1].pose.position.y - data.y)**2
                    dist_after = (trajectory.points[point_min+1].pose.position.x - data.x)**2 + (trajectory.points[point_min+1].pose.position.y - data.y)**2
                    if(dist_pre<left_distance_min_pre):
                        left_distance_min_pre =dist_pre
                        left_point_min_pre = i
                    if(dist_after<left_distance_min_after):
                        left_distance_min_after =dist_after
                        left_point_min_after = i
                        
                trajectory.points[point_min-1].pose.position.x -= (trajectory.points[point_min-1].pose.position.x - self.left_bound[left_point_min_pre].x)/1.5
                trajectory.points[point_min-1].pose.position.y -= (trajectory.points[point_min-1].pose.position.y - self.left_bound[left_point_min_pre].y)/1.5
                trajectory.points[point_min+1].pose.position.x -= (trajectory.points[point_min+1].pose.position.x - self.left_bound[left_point_min_after].x)/1.5
                trajectory.points[point_min+1].pose.position.y -= (trajectory.points[point_min+1].pose.position.y - self.left_bound[left_point_min_after].y)/1.5

                trajectory.points[point_min].pose.position.x = (trajectory.points[point_min-1].pose.position.x + trajectory.points[point_min+1].pose.position.x)/2
                trajectory.points[point_min].pose.position.y = (trajectory.points[point_min-1].pose.position.y + trajectory.points[point_min+1].pose.position.y)/2
    



                # for i in range(5):
                #     trajectory.points[point_min+i].pose.position.x -= trajectory.points[point_min+i].pose.position.x - self.left_bound[point_min+i]
            else:
                # self.get_logger().info('left object')
                right_distance_min_pre = 100
                right_distance_min_after = 100
                right_point_min_pre = -1
                right_point_min_after = -1
                for i,data in enumerate(self.right_bound):
                    dist_pre = (trajectory.points[point_min-1].pose.position.x - data.x)**2 + (trajectory.points[point_min-1].pose.position.y - data.y)**2
                    dist_after = (trajectory.points[point_min+1].pose.position.x - data.x)**2 + (trajectory.points[point_min+1].pose.position.y - data.y)**2
                    if(dist_pre<right_distance_min_pre):
                        right_distance_min_pre =dist_pre
                        right_point_min_pre = i
                    if(dist_after<right_distance_min_after):
                        right_distance_min_after =dist_after
                        right_point_min_after = i
                trajectory.points[point_min-1].pose.position.x -= (trajectory.points[point_min-1].pose.position.x - self.right_bound[right_point_min_pre].x)/1.5
                trajectory.points[point_min-1].pose.position.y -= (trajectory.points[point_min-1].pose.position.y - self.right_bound[right_point_min_pre].y)/1.5
                trajectory.points[point_min+1].pose.position.x -= (trajectory.points[point_min+1].pose.position.x - self.right_bound[right_point_min_after].x)/1.5
                trajectory.points[point_min+1].pose.position.y -= (trajectory.points[point_min+1].pose.position.y - self.right_bound[right_point_min_after].y)/1.5

                trajectory.points[point_min].pose.position.x = (trajectory.points[point_min-1].pose.position.x + trajectory.points[point_min+1].pose.position.x)/2
                trajectory.points[point_min].pose.position.y = (trajectory.points[point_min-1].pose.position.y + trajectory.points[point_min+1].pose.position.y)/2
    

            # rate = object_r / distance_min - 1
            # trajectory.points[point_min].pose.position.x += rate * (trajectory.points[point_min].pose.position.x - object_x)
            # trajectory.points[point_min].pose.position.y += rate * (trajectory.points[point_min].pose.position.y - object_y)
        
        

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