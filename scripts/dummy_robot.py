#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
A ros node to help debug path planners. Listens to PoseWithCovarianceStamped messages 
(pose setpoints) and republishes them as transforms between map and robot.
"""

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import *

class DummyRobot:

    def __init__(self,
                 initial_pose,
                 rate = 10,
                 world_frame = 'map',
                 robot_frame = 'dummy'):
        """
        rate : rate at which the transforms are published, in Hz; default is 10 Hz
        
        """
        self.rate = rate
        self.world_frame = world_frame
        self.robot_frame = robot_frame
        self.pose = initial_pose

        rospy.init_node('dummy_robot')
        self.sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.init_pose)
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.broadcast_transform)
        
    def init_pose(self, pose):
        """ Reset robot pose. """

        self.pose = pose


    def broadcast_transform(self, event):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation = self.pose.pose.pose.position
        t.transform.rotation = self.pose.pose.pose.orientation

        self.tf_br.sendTransform(t)
        
    def run(self):
        rospy.spin()
    
if __name__ == '__main__':

    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position = Point(0,0,0)
    pose.pose.pose.orientation = Quaternion(0,0,0,1)
    dummy = DummyRobot(pose)
    dummy.run()
    
