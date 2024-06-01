import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float64

from geometry_msgs.msg import Twist # https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html
from sensor_msgs.msg import NavSatFix # https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html

from rclpy.clock import Clock

class MyPublisherSubscriber(Node):

    def __init__(self):
        super().__init__('coppeliasim_ros2_interface')
        self.lat = 48.418075
        self.lon = -4.473297
        self.msg_speed = Float32()
        self.msg_speed.data = 0.0
        self.msg_steering = Float32()
        self.msg_steering.data = 0.0
        self.publisher_speed = self.create_publisher(Float32, 'speed', 10)
        self.publisher_steering = self.create_publisher(Float32, 'steering', 10)
        self.publisher_latlon = self.create_publisher(NavSatFix, 'navsatfix', 10)
        timer_20hz_period = 0.05 # 20 Hz (50 ms)
        self.timer_20hz = self.create_timer(timer_20hz_period, self.timer_20hz_callback)
        timer_1hz_period = 1 # 1 Hz (1 s)
        self.timer_1hz = self.create_timer(timer_1hz_period, self.timer_1hz_callback)
        self.subscription_cmdvel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback_cmdvel,
            10)
        self.subscription_cmdvel  # prevent unused variable warning
        self.subscription_lat = self.create_subscription(
            Float64,
            'lat',
            self.listener_callback_lat,
            10)
        self.subscription_lat  
        self.subscription_lon = self.create_subscription(
            Float64,
            'lon',
            self.listener_callback_lon,
            10)
        self.subscription_lon 

    # robot control @ 20 Hz
    def timer_20hz_callback(self):
        self.publisher_speed.publish(self.msg_speed)
        self.publisher_steering.publish(self.msg_steering)
        # self.get_logger().info('Publishing: speed = %.2f, steering = %.2f'%(self.msg_speed.data,self.msg_steering.data))

    # GPS fix every second (1Hz)
    def timer_1hz_callback(self):
        msg_nav = NavSatFix()
        msg_nav.longitude = self.lon
        msg_nav.latitude = self.lat
        time_stamp = Clock().now()
        msg_nav.header.stamp = time_stamp.to_msg()
        msg_nav.header.frame_id = "world"
        self.publisher_latlon.publish(msg_nav)
        print (self.lat, self.lon)

    # Get /cmd_vel from ROS2
    def listener_callback_cmdvel(self, msg):
        self.msg_speed.data = msg.linear.x # linear speed along x
        self.msg_steering.data = msg.angular.z # steering around z
        self.get_logger().info('Subscribing: speed: %.2f, steering: %.2f' % (self.msg_speed.data,self.msg_steering.data))

    # Get latitude and longitude from CoppeliaSim as 2 simple Float64 topics
    def listener_callback_lat(self, msg):
        self.lat = msg.data
        self.get_logger().info('Subscribing: lat: %.6f'%(self.lat))
    def listener_callback_lon(self, msg):
        self.lon = msg.data
        self.get_logger().info('Subscribing: lat: %.6f'%(self.lon))

def main(args=None):
    rclpy.init(args=args)

    my_pubsub = MyPublisherSubscriber()

    rclpy.spin(my_pubsub)
           
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
