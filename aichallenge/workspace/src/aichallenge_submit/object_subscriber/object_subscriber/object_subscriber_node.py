import math,random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from unique_identifier_msgs.msg import UUID
from autoware_auto_perception_msgs.msg import PredictedObjects,PredictedObject,PredictedObjectKinematics,ObjectClassification,Shape
from geometry_msgs.msg import Point32,Polygon,Vector3,Pose,Point,Quaternion

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/aichallenge/objects',
            self.listener_callback,
            1)
        # self.objects=[0.0,0.0,1.1,1.1]
        self.objects=[]

        self.publisher_ = self.create_publisher(PredictedObjects, '/perception/object_recognition/objects', 1)
        self.i=0
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = PredictedObjects()
        msg.header.frame_id="map"
        msg.header.stamp=self.get_clock().now().to_msg()


        msg.objects = []
        for i in range(round(len(self.objects)/4)):
            msg.objects.append(self.createPredictedObject(self.objects[i*4],self.objects[i*4+1],self.objects[i*4+2],self.objects[i*4+3]))

        self.publisher_.publish(msg)
        self.i += 1

        self.get_logger().info('Publishing objects: "%s"' % str(len(msg.objects)))
        if len(msg.objects)>0:
            self.get_logger().info('objects 0: "%s"' % msg.objects[0])
            self.get_logger().info('objects 0: "%s"' % self.objects[0:4])
    
    def createPredictedObject(self,x,y,z,w):
        predictedObject = PredictedObject()
        predictedObject.object_id.uuid = [random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255),random.randint(0, 255)]

        objectClassification = ObjectClassification()
        objectClassification.probability=1.0
        objectClassification.label=ObjectClassification.CAR
        predictedObject.classification = [objectClassification]

        predictedObject.kinematics.initial_pose_with_covariance.pose = Pose(
            position = Point(x=x,y=y,z=43.15),
            orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0) # [0,0,0,1] means not moving
        )
        
        # msg.shape = self.createPoints(x,y,z,w)
        predictedObject.shape.type=Shape.CYLINDER
        predictedObject.shape.dimensions = Vector3(x=w+0.1,y=2.0,z=2.0)

        self.get_logger().info('Shape: "%s"' % str(predictedObject.shape))

        return predictedObject

    def createPoints(self,x,y,z,w):
        point1 = Point32(); point2 = Point32(); point3 = Point32(); point4 = Point32(); 
        point1.x = x+w/2; point1.y = y; point1.z =z
        point2.x = x; point2.y = y+w/2; point2.z =z+1
        point3.x = x-w/2; point3.y = y; point3.z =z
        point4.x = x; point4.y = y-w/2; point4.z =z+1

        polygon = Polygon()
        polygon.points = [point1,point2,point3,point4,point1]
        shape = Shape()
        shape.footprint = polygon
        return shape
        

    def listener_callback(self, msg):
        self.objects=msg.data
        self.get_logger().info('Objects Size: "%s"' % len(self.objects))
        self.get_logger().info('Objects: "%s"' % self.objects)

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