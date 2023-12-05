#!/usr/bin/env python
import rospy
import random
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped 
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class RobotStates:
    def __init__(self):
        # robot rate at which commands get run
        self.running_rate = 10

        # go-to-goal target positions and angle
        self.target_goal_x = 3.0
        self.target_goal_y = 0.0
        self.theta_init = 0

        # robot output velocity
        self.velocity = 0
        self.angular_velocity = 0

        # robot measured velocity based on odom
        self.measured_linear_velocity = 0

        # robot current position and heading
        self.x = 0
        self.y = 0
        self.theta = 0

        # proportional controller constants
        self.kp = 0.15

        # info from lidar
        self.angle_range = []
        self.distance_range = []
        self.lidar_distances = []
        self.intensities = []

        self.has_waypoint = False

        # force settings
        self.eta = 15 #without waypoints is 10
        self.nu = 0.015
        self.rho_0 =0.8
        # self.space_importance = 0.005
        self.ki = 0.01
        self.kd = 0.0003
        self.ktheta = 1
        self.first_point_found = 1 
        # pid variables
        self.vref = 0.08
        self.integral = 0
        self.integral_angle = 0
        self.previous_error = 0
        self.theta_ki = 0.05

robot_states = RobotStates()
def distance(x0, y0, x1, y1):
    return math.sqrt((x1 - x0)**2 + (y1-y0)**2)

def odom_info(msg):
    global robot_states
    robot_states.measured_linear_velocity = msg.twist.twist.linear.x
    robot_states.x = round(msg.pose.pose.position.x, 2)
    robot_states.y = round(msg.pose.pose.position.y, 2)
    _, _, robot_states.theta = euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w])

def lidar_info(msg):
    global robot_states
#    print(msg)
    robot_states.angle_range = [msg.angle_min, msg.angle_max]
    robot_states.distance_range = [msg.range_min, msg.range_max]
    robot_states.lidar_distances = msg.ranges
    robot_states.intensities = msg.intensities

def waypoint_callback(msg):
    global robot_states
    #if not robot_states.has_waypoint: 
    # robot_states.target_goal_x = msg.point.x
    # robot_states.target_goal_y = msg.point.y
        # robot_states.target_goal_x = 10
        # robot_states.target_goal_y = 0
        # robot_states.has_waypoint = True
        # robot_states.eta = 150/distance(robot_states.x, robot_states.y, robot_states.target_goal_x, robot_states.target_goal_y)
        # print("robot acquired new waypoint")
    # print("waypoint already acquired")
def f_att(eta, current_x, current_y, target_x, target_y):
    return [-eta * (target_x - current_x), -eta*(target_y - current_y)]

def frep():
    global robot_states
    frep_sum = [0, 0]

    closest_distance = float('inf')
    closest_index = -1
    for i in range(0, len(robot_states.lidar_distances)):

        #if robot_states.lidar_distances[i] < closest_distance:
         #   closest_distance = robot_states.lidar_distances[i]
          #  closest_index = i

         if robot_states.lidar_distances[i] < float('inf'):
             obstacle_angle_from_robot =(robot_states.angle_range[0] + \
                                        (robot_states.angle_range[1] - robot_states.angle_range[0]) * \
                                        i / len(robot_states.lidar_distances)) + math.pi
             
             obstacle_x = robot_states.lidar_distances[i] * math.cos(obstacle_angle_from_robot + robot_states.theta) + robot_states.x 
             obstacle_y = robot_states.lidar_distances[i] * math.sin(obstacle_angle_from_robot + robot_states.theta) + robot_states.y
             # if robot_states.lidar_distances[i] < robot_states.rho_0:
             #    print("vvv")
             #    print("obstacle angle from robot:", obstacle_angle_from_robot * 180 / math.pi)
             #    print("actual angle: ", robot_states.theta * 180/math.pi)
             #    print((obstacle_angle_from_robot + robot_states.theta) * 180/math.pi)
             #    print("^^^")
                 # print(math.atan2(obstacle_y, obstacle_x) * 180/math.pi)
                 #  print([obstacle_x, obstacle_y])
             # print(obstacle_x)
             # print(obstacle_y)
             # print(robot_states.x)
             # print(robot_states.y)
             frep_point = frep_for_point(robot_states.x, robot_states.y, obstacle_x, obstacle_y, robot_states.rho_0, robot_states.nu)
        
             # print(frep_point)
             frep_sum[0] += frep_point[0]
             frep_sum[1] += frep_point[1]

    # if closest_index != -1:
    #    obstacle_angle_from_robot = robot_states.angle_range[0] + \
    #                                (robot_states.angle_range[1] - robot_states.angle_range[0]) * \
    #                                closest_index / len(robot_states.lidar_distances) - math.pi
    #    obstacle_x = robot_states.lidar_distances[closest_index] * math.cos(obstacle_angle_from_robot)
    #    obstacle_y = robot_states.lidar_distances[closest_index] * math.sin(obstacle_angle_from_robot)
    #    frep_point = frep_for_point(robot_states.x, robot_states.y, obstacle_x, obstacle_y, robot_states.rho_0,
    #                                robot_states.nu)
    #    return frep_point
    #else:
    #    return [0, 0]
    #print(frep_sum)
    return frep_sum

def frep_for_point(current_x, current_y, target_x, target_y, rho_0, nu):
    target_distance = distance(current_x, current_y, target_x, target_y)
    if target_distance > rho_0:
        return [0, 0]
    # print([target_x, target_y])
    q = [current_x, current_y]
    b = [target_x, target_y]

    # print(q)
    # print(b)
    delta = [(q[0] - b[0])/target_distance, (q[1] - b[1])/target_distance]

    # print(delta)
    multiplier = nu*((1/target_distance) - (1/rho_0)) * (1/target_distance**2)
    return [delta[0] * multiplier, delta[1]* multiplier]

#def f_space():
#    global robot_states
#    space_sum = 0
#
#    for i in range(0, len(robot_states.lidar_distances)):
#        obstacle_angle_from_robot =(robot_states.angle_range[0] + \
#                                            (robot_states.angle_range[1] - robot_states.angle_range[0]) * \
#                                            i / len(robot_states.lidar_distances)) + math.pi
#        if obstacle_angle_from_robot > 4*math.pi/3 or obstacle_angle_from_robot < 2*math.pi/3:
#            obstacle_distance = robot_states.lidar_distances[i]
#            robot_side = -1
#            if obstacle_distance == float('inf'):
#                obstacle_distance = 16
#            if obstacle_angle_from_robot > 3*math.pi/2:
#                robot_side = 1
#            space_sum += robot_states.space_importance*obstacle_distance * robot_side
#
#    return -space_sum

def main():
    global robot_states
    rospy.init_node("rosbot_gotogoal")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_info)
    # reset pose
    reset_pose = PoseWithCovarianceStamped()
    reset_pose.pose.pose.position = {0, 0, 0}
    reset_pose.pose.pose.orientation = {0,0,0,0}

    reset_pub = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=1)
    reset_pub.publish(reset_pose)

    waypoint_sub = rospy.Subscriber("/furthest_gap", PointStamped, waypoint_callback)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_info)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(robot_states.running_rate)
    velocity = Twist()
    velocity.linear.x = 0.0
    count = 0
    
    robot_states.has_waypoint = True
    while not rospy.is_shutdown():
        # # set target goal to be relative to robot's initial position
        # if count == 0:
        #     robot_states.target_goal_x = robot_states.target_goal_x + robot_states.x
        #     robot_states.target_goal_y = robot_states.target_goal_y + robot_states.y
        #     robot_states.theta_init = robot_states.theta
        #
        # count += 1
        if not robot_states.has_waypoint:
            continue

        # get updated distance from robot to goal
        distance_from_goal = math.sqrt((robot_states.target_goal_x - robot_states.x)**2 +
                                       (robot_states.target_goal_y - robot_states.y)**2)

        if distance_from_goal < 0.1 :
            robot_states.velocity = 0
            robot_states.angular_velocity = 0
            robot_states.has_waypoint = False
            #print("robot reached waypoint")
        else:
            # robot velocity pid control
            f_att = [-robot_states.eta * (robot_states.x - robot_states.target_goal_x),
                     -robot_states.eta * (robot_states.y - robot_states.target_goal_y)]
            # f_att = [0, 0]
            #print("-----------------")
            # print("f_att: ", f_att)
            f_rep = frep()
            # f_rep = [0, 0]
            # print("frep: ", f_rep)
            f_total = [f_att[0] + f_rep[0], f_att[1] + f_rep[1]] # force vector representing combined forces
            # print("ftotal: ", f_total)
            # print("-----------------")
            #print("forces x: ", f_total[0], "forces y: ", f_total[1])
            dt = 1.0 / robot_states.running_rate
            next_point_x = robot_states.x + round(f_total[0] * dt, 2)
            next_point_y = robot_states.y + round(f_total[1] * dt, 2)
            # print("current x: ", robot_states.x)
            # print("current y: ", robot_states.y)
            # print("goal x: ", robot_states.target_goal_x)
            # print("goal y: ", robot_states.target_goal_y)
            # print("next target x: ", next_point_x)
            # print("next target y: ", next_point_y)
            #print("----------------")
            #print("next point x: ", next_point_x)
            #print("next point y: ", next_point_y)
            # P controller for now
            error_from_next_point = distance(robot_states.x, robot_states.y, next_point_x, next_point_y)
            robot_states.integral = robot_states.integral + error_from_next_point * dt
            robot_states.velocity = min(robot_states.kp * error_from_next_point, 0.05)
            # robot_states.velocity = robot_states.kp * error_from_next_point + robot_states.ki * robot_states.integral
            
            #space_gamma = f_space()
            #print("space gamma: ", space_gamma)
            # robot angle to goal point
            theta_d = math.atan2((next_point_y-robot_states.y),
                                 (next_point_x-robot_states.x))
            #print("theta d: ", theta_d)
            # angle error
            e  = theta_d - (robot_states.theta)

            robot_states.integral_angle = robot_states.integral_angle + e*dt
            # robot angle proportional controller: faster turn movement
            gamma = robot_states.ktheta * math.atan2(math.sin(e), math.cos(e)) +robot_states.theta_ki * robot_states.integral_angle # + space_gamma
            if abs(gamma) > 1.0:
                robot_states.velocity = 0

            if abs(gamma) > 0.5:
                if gamma < 0:
                    gamma = -0.5
                else:
                    gamma = 0.5
            #if gamma > 1:
            #    robot_states.angular_velocity = gamma
            #    robot_states.velocity = 0
            #else:
            #    robot_states.velocity = 0.05
            #count += 1
            robot_states.angular_velocity = gamma
            #print("!!!!!!!!!!!!!!!!!!!!!!!")
            #print("Next X:" , next_point_x)
            #print("Next Y:", next_point_y)
            #print("Linear Velocity: ", robot_states.velocity)
            #print("Angular Velocity: ", robot_states.angular_velocity)
            #print("Distance to Goal: ", distance_from_goal)
            #print("Angle from next point: ", theta_d)
            #print("Current Angle: ", robot_states.theta * 180/math.pi)
            #print("Angle from goal: ", math.atan2(robot_states.target_goal_y - robot_states.y, robot_states.target_goal_x - robot_states.x))
            #print("Target goal x: ", robot_states.target_goal_x, "Target goal y: ", robot_states.target_goal_y)
            #print("Current Heading: ", robot_states.theta)
            #print("Gamma: ", gamma)
            # print("----------------")
            #print("!!!!!!!!!!!!!!!!!!!!!!!")
            #if count < 10:
            #    robot_states.velocity = 0.3
            #else:
            #    robot_states.velocity = 0
        # set new velocity to publish
        velocity.linear.x = robot_states.velocity
        velocity.angular.z = robot_states.angular_velocity
        vel_pub.publish(velocity)
        rate.sleep()


if __name__ == "__main__":
    main()



