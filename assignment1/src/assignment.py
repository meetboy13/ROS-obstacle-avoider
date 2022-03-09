#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
from numpy import nanmin
from math import isnan
from tf.transformations import euler_from_quaternion
from assignment1.msg import name

class NavigateNode():

    # Defined state constants
    MOVING_FORWARD = 0
    OBSTACLE_AVOIDING = 1
    REORIENTATING = 2
    BACKING_UP = 3

    # Define the distance an object has to be before avoiding it
    front_distance = 0.6
    tracking_distance = 1.0
    tracking_distance_max = 1.2

    # initialise variables
    state = MOVING_FORWARD
    state_count = 0
    last_state = MOVING_FORWARD
    y_pos = 0.0
    x_pos = 0.0
    last_x_pos = 0.0
    orientation = 0.0
    front_sensor = 10.0
    left_sensor = 10.0
    right_sensor = 10.0
    photo_count = 0

    def __init__(self):
        # Initialise everything - see the lab manual for descriptions
        rospy.init_node('NavigateNode', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)
        self.topic_pub=rospy.Publisher('/photo_name', name, queue_size=1)
        self.cmd_vel=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=10)
        r = rospy.Rate(10);

        # Need to listen to the laser scan, odometre and bumper
        rospy.Subscriber("/scan", LaserScan, self.OnLaserScan)
        rospy.Subscriber("/odom", Odometry, self.OnOdometer)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.OnBumper)

        # Define the standard msgs
        move_cmd = Twist()
        photo = name()

        # As long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            if NavigateNode.state == NavigateNode.MOVING_FORWARD:
                # take photo and change state
                if NavigateNode.front_sensor < NavigateNode.front_distance:
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.0
                    # take photo
                    photo = 'obstacle' + str(NavigateNode.photo_count) + '.jpg'
                    self.topic_pub.publish(photo)
                    NavigateNode.photo_count += 1
                    NavigateNode.state_count = 50
                    NavigateNode.state = NavigateNode.OBSTACLE_AVOIDING
                    NavigateNode.last_x_pos = NavigateNode.x_pos
                else:
                    move_cmd.linear.x = 0.2

                # straighten the robot's route
                if NavigateNode.y_pos < -0.6 and NavigateNode.orientation < 0.6:
                    move_cmd.angular.z = 0.2
                elif NavigateNode.y_pos > 0.6 and NavigateNode.orientation > -0.6:
                    move_cmd.angular.z = -0.2
                else:
                    move_cmd.angular.z = 0.0

            elif NavigateNode.state == NavigateNode.OBSTACLE_AVOIDING:
                if NavigateNode.state_count > 25:
                    # initial turning
                    NavigateNode.state_count -= 1
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.6
                elif NavigateNode.state_count > 15:
                    NavigateNode.state_count -= 1
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.0
                elif NavigateNode.state_count > 0:
                    if NavigateNode.front_sensor < NavigateNode.front_distance:
                        # do more turning
                        NavigateNode.state_count = 50
                        move_cmd.linear.x = 0.0
                        move_cmd.angular.z = 0.0
                    else:
                        # move forward for a bit
                        NavigateNode.state_count -= 1
                        move_cmd.linear.x = 0.6
                        move_cmd.angular.z = 0.0
                else:
                    # move around detected obstacles
                    if (abs(NavigateNode.y_pos) < 0.6) and (NavigateNode.x_pos > NavigateNode.last_x_pos):
                        NavigateNode.state = NavigateNode.REORIENTATING
                    else:
                        if NavigateNode.front_sensor < NavigateNode.front_distance:
                            move_cmd.linear.x = 0.0
                            move_cmd.angular.z = 0.6
                        elif NavigateNode.right_sensor < NavigateNode.tracking_distance:
                            move_cmd.linear.x = 0.2
                            move_cmd.angular.z = 0.3
                        elif NavigateNode.right_sensor > NavigateNode.tracking_distance_max:
                            move_cmd.linear.x = 0.2
                            move_cmd.angular.z = -0.5
                        else:
                            move_cmd.linear.x = 0.6
                            move_cmd.angular.z = 0.0


            elif NavigateNode.state == NavigateNode.REORIENTATING:
                # if not facing forward then rotate
                move_cmd.linear.x = 0.0
                if abs(NavigateNode.orientation) > 0.3:
                    move_cmd.angular.z = 0.6
                else:
                    move_cmd.angular.z = 0.0
                    NavigateNode.state = NavigateNode.MOVING_FORWARD

            elif NavigateNode.state == NavigateNode.BACKING_UP:
                # when bumper is triggered
                if NavigateNode.state_count > 20:
                    # go back
                    move_cmd.linear.x = -0.2
                    move_cmd.angular.z = 0.0
                    NavigateNode.state_count -= 1
                elif NavigateNode.state_count == 20:
                    # take photo
                    photo = 'obstacle' + str(NavigateNode.photo_count) + '.jpg'
                    self.topic_pub.publish(photo)
                    NavigateNode.photo_count += 1
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.0
                    NavigateNode.state_count -= 1
                else:
                    # turn and return to previous state
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.628
                    NavigateNode.state_count -= 1
                    if NavigateNode.state_count == 0:
                        NavigateNode.state = NavigateNode.last_state

            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def OnLaserScan(self, data):
        # update sensor readings
        count = len(data.ranges)
        NavigateNode.front_sensor = nanmin(data.ranges[count/3:count*2/3])
        NavigateNode.left_sensor = nanmin(data.ranges[count*2/3:count-1])
        NavigateNode.right_sensor = nanmin(data.ranges[0:count/3])
        NavigateNode.front_sensor = 10 if isnan(NavigateNode.front_sensor) else NavigateNode.front_sensor
        NavigateNode.left_sensor = 10 if isnan(NavigateNode.left_sensor) else NavigateNode.left_sensor
        NavigateNode.right_sensor = 10 if isnan(NavigateNode.right_sensor) else NavigateNode.right_sensor

    def OnOdometer(self, data):
        # update the position and orientation of turtlebot
        NavigateNode.y_pos = data.pose.pose.position.y
        NavigateNode.x_pos = data.pose.pose.position.x
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, NavigateNode.orientation) = euler_from_quaternion (orientation_list)

    def OnBumper(self, data):
        # change state on bumper collision
        if data.state == 1 and (NavigateNode.state != NavigateNode.BACKING_UP):
            NavigateNode.last_state = NavigateNode.state
            NavigateNode.state = NavigateNode.BACKING_UP
            NavigateNode.state_count = 40

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")

        # A default Twist has linear.x of 0 and angular.z of 0.  
        # So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # Sleep just makes sure TurtleBot receives the stop command prior to shutting
        # down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    NavigateNode()
