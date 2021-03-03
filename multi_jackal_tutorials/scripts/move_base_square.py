#!/usr/bin/env python

""" move_base_square.py - Version 1.1 2013-12-20
    Command a robot to move in a square using move_base actions..
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

from multi_control.msg import RobotStatus

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)

        robot_id = rospy.get_param("~robot_id", 0)
        # How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 1.0) # meters
        waypoints_size = rospy.get_param("~waypoints_size", 6)

        point0_x = rospy.get_param("~point0_x", 0.0)
        point0_y = rospy.get_param("~point0_y", 4.0)
        point0_theta = rospy.get_param("~point0_theta", pi/2)

        point1_x = rospy.get_param("~point1_x", -3.0)
        point1_y = rospy.get_param("~point1_y", 4.0)
        point1_theta = rospy.get_param("~point1_theta", pi)

        point2_x = rospy.get_param("~point2_x", 0.0)
        point2_y = rospy.get_param("~point2_y", 4.0)
        point2_theta = rospy.get_param("~point2_theta", 0)

        point3_x = rospy.get_param("~point3_x", 5.0)
        point3_y = rospy.get_param("~point3_y", 2.5)
        point3_theta = rospy.get_param("~point3_theta", 0)

        point4_x = rospy.get_param("~point4_x", 10.0)
        point4_y = rospy.get_param("~point4_y", 3.0)
        point4_theta = rospy.get_param("~point4_theta", 0)

        point5_x = rospy.get_param("~point5_x", 14.0)
        point5_y = rospy.get_param("~point5_y", 4.0)
        point5_theta = rospy.get_param("~point5_theta", 0)

        
        # Create a list to hold the target quaternions (orientations)
        quaternions = list()
        
        # First define the corner orientations as Euler angles
        # euler_angles = (pi/2, pi, 3*pi/2, 0)
        euler_angles = (point0_theta, point1_theta, point2_theta, point3_theta, point4_theta, point5_theta)
        
        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # Create a list to hold the waypoint poses
        waypoints = list()
        
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        # waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[0]))
        # waypoints.append(Pose(Point(square_size, square_size, 0.0), quaternions[1]))
        # waypoints.append(Pose(Point(0.0, square_size, 0.0), quaternions[2]))
        # waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))

        waypoints.append(Pose(Point(point0_x, point0_y, 0.0), quaternions[0]))
        waypoints.append(Pose(Point(point1_x, point1_y, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(point2_x, point2_y, 0.0), quaternions[2]))
        waypoints.append(Pose(Point(point3_x, point3_y, 0.0), quaternions[3]))
        waypoints.append(Pose(Point(point4_x, point4_y, 0.0), quaternions[4]))
        waypoints.append(Pose(Point(point5_x, point5_y, 0.0), quaternions[5]))
        
        # Initialize the visualization markers for RViz
        self.init_markers()
        
        # Set a visualization marker at each waypoint        
        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.goal_status_pub = rospy.Publisher('goal_status', RobotStatus, queue_size=10)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        # Initialize a counter to track waypoints
        i = 0
        timeBegin = rospy.Time.now()
        # Cycle through the four waypoints
        while i < waypoints_size and not rospy.is_shutdown():
            # Update the marker display
            self.marker_pub.publish(self.markers)
            
            # Intialize the waypoint goal
            goal = MoveBaseGoal()
            
            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'
            
            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[i]
            
            # Start the robot moving toward the goal
            self.move(goal)
            
            i += 1
        
        timeEnd = rospy.Time.now()
        time = timeEnd - timeBegin
        status_msgs = RobotStatus()
        status_msgs.Header.stamp = rospy.Time.now()
        status_msgs.is_ready = True
        self.goal_status_pub.publish(status_msgs)
        rospy.loginfo("sucess...%d,use time: %f", robot_id, time.to_sec())

        rospy.spin()
        
    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
            
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")