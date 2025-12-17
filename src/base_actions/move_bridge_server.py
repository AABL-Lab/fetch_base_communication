#! /usr/bin/env python
# movebridgeserver.py
# Joey Grossman 2025
# Joseph.Grossman@Tufts.edu

import rospy
import actionlib

from math import sin, cos, pi, atan2
from moveit_python import MoveGroupInterface, PlanningSceneInterface

import fetch_base_communication.msg
from fetch_base_communication.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import tf
from tf.transformations import euler_from_quaternion

saved_orientation = 0.0

LOCATIONS = {
    'elevators' : (52.676197052, 2.41499710083),
    '401' : (66.4334030151, 7.99283123016),
    '402' : (59.5403862, 8.19268226624),
    '481' : (63.5038337708, 12.7281942368),
    '405' : (47.4287071228, 9.14681243896),
    '420' : (37.0003814697, 9.91230010986),
    '474' : (36.3330001831, 12.8870811462),
    '472' : (34.1403961182, 18.8323631287),
    'kitchenette' : (30.0102310181, 11.7271757126),
    'huddle' : (4.02810621262, 14.8481292725),
    '435' : (23.0405063629,13.4057121277),
    'bathrooms' : (12.501241684, 14.2192783356),
    'offices near labs' : (32.1649627686, 24.7574310303),
    'offices near huddle' : (22.1733093262, 6.98352813721)
}

# Class that handles turning the robot without moving it
class TurnBaseClient(object):

    def __init__(self):
        # Starts the tf listener
        self.tf_listener = tf.TransformListener()
        # Starts the velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Turns the robot towards a coordinate without moving
    def turn(self, target_x, target_y, frame="map"):

        while not rospy.is_shutdown():

            # Defines the target point in the map frame
            target_point_map = PointStamped()
            target_point_map.point.x = target_x
            target_point_map.point.y = target_y
            target_point_map.point.z = 0.0
            target_point_map.header.frame_id = frame
            target_point_map.header.stamp = rospy.Time(0)

            try:
                # Waits for tf to receive a transform message
                self.tf_listener.waitForTransform('/base_link', frame, rospy.Time(0), rospy.Duration(1.0))
                # Transforms the target point from the map frame to the base frame
                target_point_base = self.tf_listener.transformPoint('/base_link', target_point_map)

                # Calculates the angle needed to reach the point from the transform
                angle_to_point = atan2(target_point_base.point.y, target_point_base.point.x)

                # Creates a message that will be used to send angular messages to the cmd_vel topic
                twist_msg = Twist()

                # Control function that turns the robot towards the point if the angle to the point is above 0.05 and breaks out of the while loop when it's done
                if abs(angle_to_point) > 0.05:
                    twist_msg.angular.z = 0.5 if angle_to_point > 0 else -0.5
                    self.cmd_vel_pub.publish(twist_msg)
                else:
                    break
            
            # Catches errors
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e: 
                rospy.logerr('TF Exception: %s' % e)
    
    def save_current_orientation(self, frame='map'):
        
        global saved_orientation
        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)

        current_orientation = current_pose.pose.pose.orientation
        x, y, z, w = current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w

        _, _, yaw = euler_from_quaternion([x,y,z,w])

        saved_orientation = yaw
    

    # Currently broken. Do not use
    def return_to_previous_orientation(self, frame='map'):

        while not rospy.is_shutdown():
    

            try:
                # Waits for tf to receive a transform message
                
                current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
                current_orientation = current_pose.pose.pose.orientation

                x, y, z, w = current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w

                _, _, yaw = euler_from_quaternion([x,y,z,w])

                angle_to_point = yaw - saved_orientation


                # Creates a message that will be used to send angular messages to the cmd_vel topic
                twist_msg = Twist()

                # Control function that turns the robot towards the point if the angle to the point is above 0.05 and breaks out of the while loop when it's done
                if abs(angle_to_point) > 0.05:
                    twist_msg.angular.z = 0.5 if angle_to_point > 0 else -0.5
                    self.cmd_vel_pub.publish(twist_msg)
                else:
                    break
            
            # Catches errors
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e: 
                rospy.logerr('TF Exception: %s' % e)




# Class that handles initial pose initialization
class InitialPose:

    def __init__(self):
        # Starts the initial pose publisher
        self.initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,  queue_size=0)
        rospy.sleep(1)
        # Starts the velocity publisher
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        rospy.loginfo('Initial Pose Publisher started')

    # Publishes the initial pose estimate so we don't need to manually do it
    def publish_initial_pose(self, x, y, z, w):

        msg = PoseWithCovarianceStamped()   # Msg used for setting initial position estiate
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        # Sets the parameters initial position
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(z)
        msg.pose.pose.orientation.w = float(w)

        # Apparently needed for AMCL reasons
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        # Publishes the initial pose
        self.initial_pose_publisher.publish(msg)
        rospy.loginfo('Published initial pose')
        rospy.sleep(1)

    # Rotates the robot to get a better initial estimate of where the robot is
    def rotate(self):
        rospy.loginfo('Roating to get a better estimate of location')
        vel_msg = Twist()

        angular_speed = 90.0*2*pi/360.0
        relative_angle = 6*pi

        # Linear velocity is unused
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        # Rotates clockwise
        vel_msg.angular.z = -abs(angular_speed)

        # Initial time
        t0 = rospy.Time.now().to_sec()

        # Initial rotation
        current_angle = 0


        # Continues until the robot reaches the relative angle
        while(current_angle < relative_angle):
            self.vel_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Stops the robot from rotating
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        rospy.loginfo('Finished estimating location')
        rospy.sleep(1)

# Class that handles calls to the move_base server
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected for move_base")

    # Has the base move to the amcl coordinates provided by the parameters
    def goto(self, x, y, theta, frame="map"):
        # Sets the goal parameters
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # Sends the goal to the move_base action server and waits for a response
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Class that acts as the MoveBridgeAction server
class MoveBridgeAction(object):
    _feedback = MoveBridgeFeedback()
    _result = MoveBridgeResult()

    # Initialization of the Action Server
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, fetch_base_communication.msg.MoveBridgeAction, execute_cb=self.execute_cb, auto_start = False)
        self.move_client = MoveBaseClient()
        self.turn_client = TurnBaseClient()
        self._as.start()

    # Callback function for when the server receives a goal
    def execute_cb(self, goal):
        success = True
        
        if self._as.is_preempt_requested():
            # TODO: Figure out how to do this without conflicting with the move_base server
            # Handles prempting
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        elif (goal.behavior == 'Turn'):
            # Functionality for turning towards a coordinate without moving towards it
            rospy.loginfo("%s: Turning towards %s" % (self._action_name, goal.location))
            coordinates = LOCATIONS[goal.location]
            self.turn_client.save_current_orientation()
            self.turn_client.turn(coordinates[0], coordinates[1])
            self._as.publish_feedback(self._feedback)

        elif (goal.behavior == 'Move'):
            # Functionality for base movement behavior
            coordinates = LOCATIONS[goal.location]
            rospy.loginfo("%s: Moving to %s" % (self._action_name, goal.location))
            self._as.publish_feedback(self._feedback)
            self.move_client.goto(coordinates[0], coordinates[1], 0)

        elif (goal.behavior == 'Initialize'):
            # Sets the initial position and rotates to get a better estimate of location
            initial_pose_publisher = InitialPose()
            initial_pose_publisher.publish_initial_pose(32.137878418, 24.9275684357, -0.598218430649, 0.80133308257) # sets the initial location to the intersection close to the AABL lab
            #initial_pose_publisher.rotate()
            rospy.sleep(1)

        else:
            # If the behavior doesn't match any of the programmed behaviors
            success = False
            self._result.result = 'Invalid Behavior'
            rospy.loginfo('%s: Invalid Behavior' % self._action_name)
            self._as.set_aborted(self._result)
            
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)        


if __name__ == '__main__':
    rospy.init_node('move_bridge')
    server = MoveBridgeAction(rospy.get_name())
    rospy.spin()