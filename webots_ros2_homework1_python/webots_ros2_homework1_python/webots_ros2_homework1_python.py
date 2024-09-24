import rclpy
# Import the ROS2 python libraries
from rclpy.node import Node
# Import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# Import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# Import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time
import csv

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.37
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.78
# LIDAR_AVOID_DISTANCE = 0.5
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90
FILENAME = f'trial-{time.time()}.csv'

STALL_THRESHOLD = 0.017
STALL_TIME_LIMIT = 7

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.is_start_phase = True
        self.scan_cleaned = []
        self.stall = False
        self.last_position = None
        self.stall_time = 0
        self.stall_count = 0
        self.pose_saved = None
        # self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        self.cmd = Twist()

        # Timer for data storage loop
        self.data_timer = self.create_timer(1, self.data_timer_callback)

        # Timer for movement loop
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def data_timer_callback(self):
        if not self.pose_saved:
            return
        with open(FILENAME, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.pose_saved.x, self.pose_saved.y])

    def listener_callback1(self, msg1):
        # self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        # self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        # orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        # (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        if not self.pose_saved:
            self.pose_saved = position
            return
        
        # Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        # delta_x = math.fabs(self.pose_saved.x - position.x)
        # delta_y = math.fabs(self.pose_saved.y - position.y)
        # distance_moved = math.sqrt(delta_x**2 + delta_y**2)

        # if distance_moved < STALL_THRESHOLD:
        #     self.stall_time += 0.5
        #     if self.stall_time > STALL_TIME_LIMIT:
        #         self.stall = True
        # else:
        #     self.stall_time = 0
        #     self.stall = False
        self.pose_saved = position

        # if delta_x < 0.0001 and delta_y < 0.0001:
        #     self.stall = True
        # else:
        #     self.stall = False
           
        return None
        
    def timer_callback(self):
        # if (len(self.scan_cleaned)==0):
        #     self.turtlebot_moving = False
        #     return
        
        # left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        # right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        # front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        # self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        # self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))

        # Angular z - rotates CW (right) when negative, CCW (left) when positive
        # Linear x - moves forward when positive, backward when negative

        if not self.last_position:
            self.last_position = (left_lidar_min, front_lidar_min, right_lidar_min)
        else:
            left_mvmt = math.fabs(self.last_position[0] - left_lidar_min)
            front_mvmt = math.fabs(self.last_position[1] - front_lidar_min)
            right_mvmt = math.fabs(self.last_position[2] - right_lidar_min)
            self.get_logger().info(f'left_lidar_min: {left_lidar_min}, left_mvmt: {left_mvmt}')
            self.get_logger().info(f'front_lidar_min: {front_lidar_min}, front_mvmt: {front_mvmt}')
            self.get_logger().info(f'right_lidar_min: {right_lidar_min}, right_mvmt: {right_mvmt}')
            self.get_logger().info(f'self.stall_time: {self.stall_time}')

            if left_mvmt < STALL_THRESHOLD and front_mvmt < STALL_THRESHOLD and right_mvmt < STALL_THRESHOLD:
                self.stall_time += 1
                if self.stall_time > STALL_TIME_LIMIT:
                    self.stall = True
            else:
                self.stall_time = 0
                self.stall = False
            self.last_position = (left_lidar_min, front_lidar_min, right_lidar_min)
        
        # Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        # delta_x = math.fabs(self.pose_saved.x - position.x)
        # delta_y = math.fabs(self.pose_saved.y - position.y)
        # distance_moved = math.sqrt(delta_x**2 + delta_y**2)

        if self.stall:
            self.cmd.linear.x = -0.2
            self.cmd.angular.z = 0.5
            self.publisher_.publish(self.cmd)
            self.stall_count = 5
            self.stall_time = 0

        if self.stall_count:
            self.cmd.angular.z = 0.3
            if self.stall_count > 2:
                self.cmd.linear.x = -0.2
            else:
                self.cmd.linear.x = 0.2
            self.publisher_.publish(self.cmd)
            self.stall_count -= 1
            self.stall_time = 0

        elif self.is_start_phase:
            self.cmd.angular.z = 0.0
            if front_lidar_min < 0.8:
                self.is_start_phase = False
                self.cmd.linear.x = 0.0
            else:
                self.cmd.linear.x = 0.2
            self.publisher_.publish(self.cmd)

        elif front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.2
            self.publisher_.publish(self.cmd)
        
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.1
            if right_lidar_min > left_lidar_min:
                self.cmd.angular.z = -0.2
            self.publisher_.publish(self.cmd)

        else:
            if right_lidar_min < 0.53:
                if front_lidar_min > 0.5:
                    self.cmd.linear.x = 0.2
                else:
                    self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.2
            elif right_lidar_min > 0.8:
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = -0.25
            elif right_lidar_min > 0.6:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = -0.2
            else:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0

            self.publisher_.publish(self.cmd)
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 
def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # Pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # Shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
