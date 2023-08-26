#!/usr/bin/env python

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

class Waypoints():
    def __init__(self):
        rospy.init_node('waypoint')
        self.topic_name = "move_base_simple/goal"
        self.new_points = list()
        self.current_sequence = 0
        try:
            self.new_points = rospy.get_param('waypoints/points')
        except KeyError:
            self.start_record_points = True
            self.end_record_points = False
            rospy.logerr("No points have provided, listening from " + str(self.topic_name) + " instead")
            self.sub = rospy.Subscriber(self.topic_name, PoseStamped, self.rviz_cb)
        
        if(self.start_record_points):
            #dont do anything until it finishes record the points needed
            self.record()
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")

        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("move_base Action Server is not available")
            rospy.signal_shutdown("move_base Action server is not available")
            return
        rospy.loginfo("Connected to the move_base server")
        rospy.loginfo("Starting navigation to points...")
        self.start_nav(self.current_sequence)
        
    def active_cb(self):
        rospy.logdebug("Robot currently navigating to the next destination")
    
    def feedback_cb(self, feedback):
        rospy.logdebug("Feedback is received: " + str(feedback))
    
    def done_cb(self, status, result):
        if status == 0:
            #Request pending
            rospy.loginfo(str(result))
        
        if status == 1:
            #Request Active
            rospy.loginfo(str(result))

        if status == 2:
            #Request preempted
            rospy.loginfo(str(result))
        
        if status == 3:
            #A Goal is reached
            rospy.loginfo(str(result))
            self.current_sequence += 1
            if self.current_sequence > self.goal_cnt - 1:
                rospy.loginfo("All destination have been reached, repeat waypoints?[Y/n]")
                wait = input()
                if wait == 'y' or 'Y':
                    rospy.loginfo("Redo destination record?[Y/n]")
                    wait = input()
                    if wait == 'Y' or 'y':
                        self.goal_cnt = 0
                        self.new_points = list()
                        self.record()
                        self.current_sequence = 0
                        self.start_nav(self.current_sequence)
                    else :
                        self.current_sequence = 0
                        self.start_nav(self.current_sequence)
                else:
                    rospy.signal_shutdown("Navigation done")
            else:
                self.start_nav(self.current_sequence)
        
        if status == 4:
            #A Goal is aborted by the action server
            rospy.loginfo(str(result))
        
        if status == 5:
            #Goal pose is rejected by the action server
            rospy.loginfo(str(result))

        if status == 6:
            #Goal is now preempting
            rospy.loginfo(str(result))
        
        if status == 7:
            #Goal is recalling
            rospy.loginfo(str(result))

        if status == 8:
            #Goal pose (thiscount) received a cancel before started executing, cancelled!
            rospy.loginfo(str(result))
        
        if status == 9:
            #Goal is lost
            rospy.logerr("Cancel request fulfilled. Goal is fulfilled!")

    def record():
        message = rospy.wait_for_message(self.topic_name, PoseStamped, rospy.Duration(5.0))
        if not message:
            rospy.logdebug("Too long to wait the message. This node is automatically destroyed")
            rospy.signal_shutdown("No message received in " + rospy.Duration(5.0))
            
        wait = 'n'
        while wait != 'y':
            rospy.loginfo("Enter 'y' to stop recording waypoints")
            wait = input()
        
        if len(self.new_points) == 0:
            rospy.loginfo("No points added, terminating this node")
            rospy.signal_shutdown("Im useless now :(")
        
        rospy.loginfo(str(len(self.new_points)) + " Points have been added")
        self.goal_cnt = len(self.new_points)
        rospy.loginfo("Successfully recorded the waypoints, sending it to move_base...")
        rospy.set_param('points', self.new_points)
        self.end_record_points = True
        self.start_record_points = False
        #TODO: output the recorded points to yaml
    
    def start_nav(self, index = 0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.new_points[index]
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        if index == 0:
            rospy.loginfo("First destination is sent")
        else:
            rospy.loginfo("Destination " + str(index) + " is sent")
    
    def rviz_cb(self, data):
        
        if(self.end_record_points != True):
            temporary = PoseStamped()
            temporary = data
            self.new_points.append(temporary.pose)            
            rospy.loginfo("Point added: " + str(self.new_points[-1]))

        else:
            rospy.loginfo("Getting a point but currently not recording points")

if __name__ == '__main__':
    try:
        Waypoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoints cancelled")
        