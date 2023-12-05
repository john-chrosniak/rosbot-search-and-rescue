#!/usr/bin/env python
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import tf2_ros
from scipy.spatial.transform import Rotation

class RobotStates:
    def __init__(self):
        # robot rate at which commands get run, in Hz
        self.running_rate = 10

        # robot current position and heading
        self.x = 0
        self.y = 0
        self.quat = [0, 0, 0, 1]

        # speed/angle control
        self.velocity = 0
        self.angular_velocity = 0

        # lidar measurements
        self.angle_range = []
        self.distance_range = []
        self.angle_increment = 0
        self.lidar_distances = []
        self.intensities = []

class OccupancyGridInfo:
    def __init__(self):
        # resolution of map in meters
        self.resolution = 0.01

        # width, height of map in meters/resolution
        self.width = 500 # 10m
        self.height = 500 # 10m

        # size of robot as a square (longest side), in meters
        self.robot_square_size = 0.2

        # constants for log-odds
        self.log_odd_occupied = 0.7 # from slides
        self.log_odd_free = 0.1
        self.max_log_odd_value = 10 # max log_odd sum 
        # occupancy map representation
        self.grid = np.zeros(1)
        self.occupied_cells = np.array([])


# initialization of first states
robot_states = RobotStates()
occupancy_grid_info = OccupancyGridInfo()
tf_buffer = None
tf_listener = None
occupancy_pub = None

# get information from odometer
def odom_info(msg):
    global robot_states
    robot_states.x = msg.pose.pose.position.x
    robot_states.y = msg.pose.pose.position.y
    robot_states.quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    

# get information from lidar scans
def lidar_info(msg):
    global robot_states
    global occupancy_grid_info
    global tf_buffer
    try:
        transform = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs))
    except:
        return
    rotation_matrix = np.eye(4, dtype=np.float32)
    yaw_angle = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]).as_euler('zxy')[0]
    rotation_matrix[0:3,0:3] = Rotation.from_euler('z', yaw_angle).as_dcm().astype(np.float32)
    rotation_matrix[0:3,3] = np.asarray([transform.transform.translation.x, transform.transform.translation.y, 0.0])
    rotation_matrix.transpose(0,1)
    angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
    ranges = np.array(msg.ranges)
    # print("Num infs:", np.sum(np.isinf(msg.ranges)))
    ranges[np.isinf(msg.ranges)] = 16.0
    # print(ranges)
    # print(float("inf"))
    lidar_robot = np.array([-1*np.cos(angles) * ranges, -1*np.array(np.sin(angles) * ranges), np.zeros(len(ranges)), np.ones(len(ranges))])
    lidar_world = np.matmul(rotation_matrix, lidar_robot)[:2].T
    occupancy_grid_info.occupied_cells = (lidar_world / occupancy_grid_info.resolution).astype(np.int32)

    # for i in range(0, len(robot_states.lidar_distances)):
    #     d = robot_states.lidar_distances[i]
    #     if d >= float('inf'):
    #         d = 20
    #     obstacle_angle_from_robot = robot_states.angle_range[0] + robot_states.angle_increment * i # + math.pi
    #     try:
    #         # print("--------")
    #         # print("distance: ", d)
    #         obstacle_x = (d * math.cos(obstacle_angle_from_robot + robot_states.theta) + robot_states.x) 
    #         # print("original x: ", obstacle_x)
    #         obstacle_x = int(obstacle_x / occupancy_grid_info.resolution)
    #         # print("new x: ", obstacle_x)
    #         obstacle_y = (d * math.sin(obstacle_angle_from_robot + robot_states.theta) + robot_states.y) 
    #         # print("obstacle y: ", obstacle_y)
    #         obstacle_y = int(obstacle_y / occupancy_grid_info.resolution)
    #         # print("new y: ", obstacle_y)
    #         lidar_obstacle_points.append((obstacle_x, obstacle_y))
    #     except ValueError as e:
    #         print(e)
    #         print("angle range start: ", robot_states.angle_range[0])
    #         print("angle increment: ", robot_states.angle_increment)
    #         print("obstacle angle: ", obstacle_angle_from_robot)
    #         print("current theta: ", robot_states.theta)
    #         raise ValueError("NaN to integer")
    

# bresenham's line drawing algorithm for slopes < 1
# (x0, y0) is robot point, rounded down to int
# (x1, y1) is lidar point, rounded down to int
# taken from Wikipedia's pseudocode
def plot_line_low(x0, y0, x1, y1):
    # print("----")
    # print("from: ", (x0, y0))
    # print("to: ", (x1, y1))
    global occupancy_grid_info
    dx = x1 - x0
    dy = y1 - y0
    y_i = 1
    if dy < 0:
        y_i = -1
        dy = -dy
    # print(dy)

    D = (2 * dy) - dx
    y = y0
    x_range = np.arange(x0, x1, 1, dtype=int)
    for x in x_range:
        # print("adding here: ", (y, x))
        # print("D: ", D)
        add_value_to_grid(x, y, -occupancy_grid_info.log_odd_free)
        if D > 0:
            y = y + y_i
            D = D + (2 * (dy - dx))
        else:
            D = D + 2 * dy


# bresenham's line drawing algorithm for slopes > 1
# (x0, y0) is robot point, rounded down to int
# (x1, y1) is lidar point, rounded down to int
# taken from Wikipedia's pseudocode
def plot_line_high(x0, y0, x1, y1):
    # print("----")
    # print("from: ", (x0, y0))
    # print("to: ", (x1, y1))
    global occupancy_grid_info
    dx = x1 - x0
    dy = y1 - y0
    x_i = 1
    if dx < 0:
        x_i = -1
        dx = -dx
    # print(x1)
    # print(x0)
    # print(x1 - x0)
    # print(dx)
    D = (2 * dx) - dy
    x = x0
    y_range = np.arange(y0, y1, 1, dtype=int)
    # print(y_range)
    for y in y_range:
        # print("adding here: ", (y, x))
        # print("D: ", D)
        add_value_to_grid(x, y, -occupancy_grid_info.log_odd_free)
        if D > 0:
            x = x + x_i
            D = D + (2 * (dx - dy))
        else:
            D = D + 2 * dx


# complete Bresenham's algo
# taken from Wikipedia pseudocode
def plot_line(x0, y0, x1, y1):
    if np.abs(y1 - y0) < np.abs(x1 - x0):
        if x0 > x1:
            # return
            plot_line_low(x1, y1, x0, y0) # this line makes the vertical lines
        else:
            plot_line_low(x0, y0, x1, y1)
    else:
        if y0 > y1:
            plot_line_high(x1, y1, x0, y0)
        else:
            plot_line_high(x0, y0, x1, y1)


# adds the log_odds value of cell (x, y) if cell is within bounds of the grid space
# adds up until it reaches a maximum log-odds value (to stop it from continuously adding infinitely)
def add_value_to_grid(x, y, log_odds_value):
    global occupancy_grid_info
    
    offset_x = int(occupancy_grid_info.width // 2)
    offset_y = int(occupancy_grid_info.height // 2)

    if  0 <= x + offset_x < occupancy_grid_info.width  and  0 <= y + offset_y < occupancy_grid_info.height:
        if np.abs(occupancy_grid_info.grid[y + offset_y][x + offset_x] + log_odds_value) < occupancy_grid_info.max_log_odd_value:
            # if log_odds_value < 0:
                # print("------")
                # print("adding free value at this location: ", (y + offset_y, x + offset_x))
            occupancy_grid_info.grid[y + offset_y][x + offset_x] += log_odds_value
            # occupancy_grid_info.grid[x + offset_x][y + offset_y] += log_odds_value

            

def main():
    global robot_states, tf_buffer, tf_listener, occupancy_pub
    rospy.init_node("occupancy_grid")
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(0, 1e8))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_info)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_info, queue_size=1)
    occupancy_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=8)

    # reset pose
    reset_pose = PoseWithCovarianceStamped()
    reset_pose.pose.pose.position = {0, 0, 0}
    reset_pose.pose.pose.orientation = {0,0,0,0}

    reset_pub = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=1)
    reset_pub.publish(reset_pose)

    # grid messasge
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.info.resolution = occupancy_grid_info.resolution
    occupancy_grid_msg.info.width = occupancy_grid_info.width
    occupancy_grid_msg.info.height = occupancy_grid_info.height
    occupancy_grid_msg.header.frame_id = 'odom'


    # 0 = free
    # 1 = occupied
    # -1 = unknown
    occupancy_grid_msg.data = range(occupancy_grid_info.width * occupancy_grid_info.height)

    # origin for occupancy grid; real world pose where origin (0,0) is
    occupancy_grid_msg.info.origin.position.x = -occupancy_grid_info.width//2 * occupancy_grid_info.resolution
    occupancy_grid_msg.info.origin.position.y = -occupancy_grid_info.height//2 * occupancy_grid_info.resolution
    # occupancy_grid_info.info.origin.position.theta = 0

    rate = rospy.Rate(robot_states.running_rate)

    # grid representation as a np 2D array
    # can be any value; centered around 0
    # uses log-odds for occupied and free? constant measurements

    occupancy_grid_info.grid = np.zeros((occupancy_grid_info.height, occupancy_grid_info.width)) # start as unknown

    # occupancy_grid_info.grid[occupancy_grid_info.height//2 - 10: occupancy_grid_info.height//2 + 10][occupancy_grid_info.width//2 - 10: occupancy_grid_info.width + 10] = 0.5

    while not rospy.is_shutdown():
        lidar_points = occupancy_grid_info.occupied_cells.copy()
        
        for x, y in lidar_points:
            x_round = int(robot_states.x / occupancy_grid_info.resolution)
            y_round = int(robot_states.y / occupancy_grid_info.resolution)
            # print("----")
            # print("robot position: ", (x_round, y_round))
            # print("lidar point: ", (x, y))
            plot_line(x_round, y_round, x, y)
            add_value_to_grid(x, y, occupancy_grid_info.log_odd_occupied)
        for i in range(occupancy_grid_info.height * occupancy_grid_info.width):
            if occupancy_grid_info.grid.flat[i] < 0:
                occupancy_grid_msg.data[i] = 0
            elif occupancy_grid_info.grid.flat[i] > 0:
                occupancy_grid_msg.data[i] = 1
            else:
                occupancy_grid_msg.data[i] = -1

        try:
            occupancy_pub.publish(occupancy_grid_msg)
        except Exception as e:
            print(e)
            # print(occupancy_grid_msg.data)
            break

        occupancy_grid_msg.header.stamp = rospy.Time.now()
        rate.sleep()

if __name__ == "__main__":
    main()

