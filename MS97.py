#!/usr/bin/env python
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import rospy
import time

CMD_VEL_PUB = None

Front = 0
Left = 0
FLeft = 0
FRight = 0
Right = 0

ROBOT_POS = Point()
ROBOT_ANG = 0

MIN_ANG_GAP = math.pi / 90
MIN_DIS_GAP = 0.3

DES_POS1 = Point()
DES_POS1.x = 2.0
DES_POS1.y = -0.7
DES_POS1.z = 0

DES_POS2 = Point()
DES_POS2.x = 3.8
DES_POS2.y = -0.7
DES_POS2.z = 0

DES_POS3 = Point()
DES_POS3.x = 4.1
DES_POS3.y = -0.7
DES_POS3.z = 0

DOOR1 = 0
DOOR2 = 0

regions_ = {
    'right': 0,
    'fright': 0,
    'fleft': 0,
    'left': 0,
    'front': 0,
}

state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'maze end',
    4: 'stop',
    5: 'turnRight',
    6: 'turnLeft',
    7: 'straight'
}




STATE = 0
SECTION = 0

TS = 0
TD = 0

# Callbacks -->


def ODOM_CALLBACK(msg):
    global ROBOT_ANG
    global ROBOT_POS
    ROBOT_POS = msg.pose.pose.position
    odom_ori = msg.pose.pose.orientation
    euler = euler_from_quaternion(
        [odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w])
    ROBOT_ANG = euler[2]

# GEZIBO's State -->


def CHANGE_STATE(new_state):
    global STATE
    STATE = new_state


def NORMALIZE_ANGLE(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def DONE_MOVING():
    command = Twist()
    command.linear.x = 0
    command.angular.z = 0
    CMD_VEL_PUB.publish(command)


def FIX_ANG(des_pos):
    global ROBOT_ANG
    global CMD_VEL_PUB
    global MIN_ANG_GAP
    destination_yaw = math.atan2(
        des_pos.y - ROBOT_POS.y, des_pos.x - ROBOT_POS.x)
    err_yaw = NORMALIZE_ANGLE(destination_yaw - ROBOT_ANG)
    command = Twist()
    if math.fabs(err_yaw) > MIN_ANG_GAP:
        command.angular.z = 0.3 if err_yaw > 0 else -0.3
    CMD_VEL_PUB.publish(command)
    if math.fabs(err_yaw) <= MIN_ANG_GAP:
        CHANGE_STATE(1)

# Odometer Callbacks -->


def ODOM_ROTATION(des_angle_in_radian, kp):
    global ROBOT_ANG
    err_yaw = NORMALIZE_ANGLE(des_angle_in_radian - ROBOT_ANG)
    return kp * err_yaw


def GO_DIRECTLY(des_pos):
    global ROBOT_ANG
    global CMD_VEL_PUB
    global MIN_ANG_GAP
    global MIN_DIS_GAP
    destination_yaw = math.atan2(
        des_pos.y - ROBOT_POS.y, des_pos.x - ROBOT_POS.x)
    err_yaw = destination_yaw - ROBOT_ANG
    err_pos = math.sqrt(pow(des_pos.y - ROBOT_POS.y, 2) +
                        pow(des_pos.x - ROBOT_POS.x, 2))
    if err_pos > MIN_DIS_GAP:
        command = Twist()
        command.linear.x = 0.2  # m/s
        command.angular.z = 0.2 if err_yaw > 0 else -0.2  # rad/s.
        CMD_VEL_PUB.publish(command)
    else:
        CHANGE_STATE(2)
    if math.fabs(err_yaw) > MIN_ANG_GAP:
        CHANGE_STATE(0)

# Control the LIDAR -->


def LASER_RANGE(msg):
    global Front, FLeft, Left, Right, FRight
    Front = min(min(msg.ranges[0:5]), min(msg.ranges[355:]))
    FLeft = min(msg.ranges[15:65])
    Left = min(msg.ranges[74:105])
    Right = min(msg.ranges[268:271])
    FRight = min(msg.ranges[299:345])

# Main Function


def MAIN():
    global CMD_VEL_PUB, SECTION
    global DOOR1, DOOR2, STATE
    global TS, TD
    global Front, FLeft, Left, Right, FRight
    global MIN_ANG_GAP
    CMD_VEL_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('/scan', LaserScan, LASER_RANGE)
    odom_sub = rospy.Subscriber('/odom', Odometry, ODOM_CALLBACK)

    rospy.init_node('maze_navigation')
    rate = rospy.Rate(10)

    # Stop the robot -->
    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0

    time.sleep(1)  # wait for node to initialize
    near_wall = 0  # start with 0, when we get to a wall, change to 1
    distance = 0.4
    print("Turning...")
    command.angular.z = -0.5
    command.linear.x = 0.1
    CMD_VEL_PUB.publish(command)
    time.sleep(2)

    while not rospy.is_shutdown():
        if(SECTION == 1):
            if(STATE == 0):
                FIX_ANG(DES_POS1)
            elif(STATE == 1):
                GO_DIRECTLY(DES_POS1)
            else:
                SECTION = 2
                STATE = 0
            if(math.isinf(Left) and math.isinf(Front)) and math.isinf(Right):
                DOOR1 = DOOR1 + 1
            if(DOOR1 > 1):
                DOOR1 = 1
                print("Door 1 found")
            print("section >> 01")
        elif(SECTION == 2):
            if(STATE == 0):
                FIX_ANG(DES_POS2)
            elif(STATE == 1):
                GO_DIRECTLY(DES_POS2)
            else:
                SECTION = 4
                STATE = 0
            if(math.isinf(Left) and math.isinf(Front)) and math.isinf(Right):
                DOOR2 = DOOR2 + 1
            if(DOOR2 > 1):
                DOOR2 = 1
                print("Door 2 found")
            print("section >> 02")
            print("Total Door count: " + str(DOOR1 + DOOR2))
        
        elif(SECTION == 4):
            print("section4")
            TD = DOOR1 + DOOR2
            print("Total door count : "+str(TD))
            if(TS == 0 and TD == 1):
                
                
                    
            # Cylinder Following Part --> 
            elif(SECTION == 3):
            print("Moving towards Cylinder")
            print("Total Door count: " + str(DOOR1 + DOOR2))
            
            if(STATE == 0):
                FIX_ANG(DES_POS3)
            elif(STATE == 1):
                GO_DIRECTLY(DES_POS3)
            else:
                if (DOOR1 + DOOR2 == 1):
                    print("Clockwise")
                    
                    def turn_right():
                    global regions_
                    
                    regions_ = {
                               'right': min(msg.ranges[-90], 10),
                               'fright': min(min(msg.ranges[-45:-1]), 10),
                               'front': min(msg.ranges[0], 10),
                               'fleft': min(min(msg.ranges[1:45]), 10),
                               'left': min(msg.ranges[90], 10),
        
                               }
                    
                    regions = regions_
                    msg = Twist()
                    time.sleep(6)
                    msg.angular.z = -0.5
                    msg.linear.x = 0.5
                    publisher_.publish(msg)
                    time.sleep(6)
                    msg.angular.z = 0.0
                    msg.linear.x = 0.0
                    publisher_.publish(msg)
                    time.sleep(10)
                    return msg
                    
                    
                elif (DOOR1 + DOOR2 == 2):
                    print("Counter-clockwise")
                    
                    def turnleft():
                    msg = Twist()
                    time.sleep(6)
                    msg.angular.z = 0.5
                    msg.linear.x = 0.5
                    publisher_.publish(msg)
                    time.sleep(6)
                    msg.angular.z = 0.0
                    msg.linear.x = 0.0
                    publisher_.publish(msg)
                    return msg
                    
                       
                    
                    
                
            elif(TS == 0 and TD == 2):
                print("Cylinder Following")
                
            else:
                # print("Doors are not found.")
                DONE_MOVING()
        else:
            # Searching for a wall
            while(near_wall == 0 and not rospy.is_shutdown()):  # wall following
                if(Front > distance and FRight > distance and FLeft > distance):  # Nothing there, go straight
                    command.angular.z = -0.1
                    command.linear.x = 0.22
                elif(FLeft < distance):
                    near_wall = 1
                else:
                    command.angular.z = -0.25
                    command.linear.x = 0.0

                CMD_VEL_PUB.publish(command)
            # Already found a wall
            else:
                if(Front > distance):
                    if(FRight < (distance / 2)):
                        print(
                            "Range: {:.2f}m - Too close. Backing up.".format(FRight))
                        command.angular.z = +1.2
                        command.linear.x = -0.1
                    elif(FRight > (distance * 0.75)):
                        print(
                            "Range: {:.2f}m - Wall-following; turn left.".format(FRight))
                        command.angular.z = -0.8
                        command.linear.x = 0.22
                    else:
                        print(
                            "Range: {:.2f}m - Wall-following; turn right.".format(FRight))
                        command.angular.z = +0.8
                        command.linear.x = 0.22
                else:
                    print("Front obstacle detected. Turning away.")
                    command.angular.z = +1.0
                    command.linear.x = 0.0
                    CMD_VEL_PUB.publish(command)
                    while(Front < 0.3 and not rospy.is_shutdown()):
                        CMD_VEL_PUB.publish(command)
                CMD_VEL_PUB.publish(command)
        if(math.isinf(Front) and math.isinf(FLeft) and not math.isinf(Right)):
            if(not SECTION == 2 and not SECTION == 3 and not SECTION == 4 and not SECTION == 5):
                SECTION = 1
        rate.sleep()


if __name__ == '__main__':
    MAIN()
