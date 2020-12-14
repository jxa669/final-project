#!/usr/bin/python

"""
This is the main entry point for the kalman filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.KFLocaliser() to do the localisation.
"""

import rospy
import supermarket
import path_finding.a_star
from path_finding.util import *
from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped, Pose )
from tf.msg import tfMessage
from nav_msgs.msg import (OccupancyGrid, Path)
from threading import Lock
from time import sleep

import sys

class AStarNode(object):
    def __init__(self):
        # ----- Minimum change (m/radians) before publishing new pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)  
        
        self._a_star = path_finding.a_star.AStar()
        self._soundhandle = SoundClient()
        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False
        #TODO Change path publisher to different type
        self._path_publisher = rospy.Publisher("/path", Path)
        '''
        self._pose_publisher = rospy.Publisher("/estimatedpose", PoseStamped)
        self._amcl_pose_publisher = rospy.Publisher("/amcl_pose",
                                                    PoseWithCovarianceStamped)
        self._tf_publisher = rospy.Publisher("/tf", tfMessage)
        '''
        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        self._a_star.set_map(ocuccupancy_map)
        
        #TODO subscribe to correct topics
        self._goal_pose_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                                                  self._goal_pose_callback)
        self._goal_pose_subscriber = rospy.Subscriber("/direction", PoseStamped,
                                                  self._goal_pose_callback)
        self._initial_pose_subscriber = rospy.Subscriber("/initialpose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)
        self._initial_pose_subscriber = rospy.Subscriber("/amcl_pose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)

    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._a_star.set_initial_pose(pose)
        #TODO change this
        #self._last_published_pose = deepcopy(self._a_star.estimatedpose)
        self._initial_pose_received = True
    
    def _simple_goal_pose_callback(self, pose):
        new_pose = PoseStamped()
        new_pose.pose = pose
        new_pose.header.frame_id = "/map"
        self._goal_pose_callback(new_pose)
    
    def _goal_pose_callback(self, pose):
        """
        Goal pose recieved
        """
        if self._initial_pose_received:
        
            path_time = self._a_star.calculate_path_to(pose);
            directions_time = self._a_star.calculate_directions();
        
            rospy.loginfo("Done pathfinding in %f", path_time)
            rospy.loginfo("Done directions in %f", directions_time)
            # ----- Publish the new path
            self._path_publisher.publish(self._a_star.calculated_path)
            x = ""
            for direction in self._a_star.directions:
                self._soundhandle.voiceSound(direction, 1.0).play()
                sleep(0.13 * len(direction))
            ''''
            estimatedpose =  PoseStamped()
            estimatedpose.pose = self._a_star.estimatedpose.pose.pose
            estimatedpose.header.frame_id = "map"
            self._pose_publisher.publish(estimatedpose)
            '''
    
            # ----- Get updated transform and publish it
            #self._tf_publisher.publish(self._a_star.tf_message)

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("path_finding")
    node = AStarNode()
    rospy.spin()
