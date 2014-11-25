#!/usr/bin/env python

import rospy

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store_msgs.msg import StringList
from mongodb_store.message_store import MessageStoreProxy

from task_executor.task_routine import delta_between
from routine_behaviours.patrol_routine import PatrolRoutine, create_patrol_task

import random


def create_simple_follow_task(waypoint_name, duration=rospy.Duration(300)):
    task = Task(start_node_id=waypoint_name, end_node_id=waypoint_name, action='simple_follow', max_duration=duration)
    task_utils.add_int_argument(task, duration.secs)
    return task

def create_rgbd_record_task(waypoint_name, duration=rospy.Duration(30), camera='head_xtion', with_depth=True, with_rgb=True):
    task = Task(start_node_id=waypoint_name, end_node_id=waypoint_name, action='record_camera', max_duration=duration)
    task_utils.add_string_argument(task, camera)
    task_utils.add_bool_argument(task, with_depth)
    task_utils.add_bool_argument(task, with_rgb)
    return task

def create_3d_scan_task(waypoint_name):
    task = Task(start_node_id=waypoint_name, end_node_id=waypoint_name, action='ptu_pan_tilt_metric_map', max_duration=rospy.Duration(240))
    task_utils.add_int_argument(task, '-160')
    task_utils.add_int_argument(task, '20')
    task_utils.add_int_argument(task, '160')
    task_utils.add_int_argument(task, '-25')
    task_utils.add_int_argument(task, '25')
    task_utils.add_int_argument(task, '25')
    return task

def create_tweet_task(waypoint_name, tweet, image_topic):
    task = Task(start_node_id=waypoint_name, end_node_id=waypoint_name, action='strands_image_tweets', max_duration=rospy.Duration(60))

    task_utils.add_string_argument(task, tweet)
    task_utils.add_bool_argument(task, False)
    task_utils.add_string_argument(task, image_topic)
    return task


def create_mongodb_store_task(db, to_replicate, delete_after_move=True):
    task = Task()
    # no idea, let's say 2 hours for now -- this action server can't be preempted though, so this is cheating
    task.max_duration = rospy.Duration(60 * 60 * 2)
    task.action = 'move_mongodb_entries'

    # replicate from this db
    task_utils.add_string_argument(task, db)

    # add arg for collectionst o replication
    collections = StringList(to_replicate)
    msg_store = MessageStoreProxy()
    object_id = msg_store.insert(collections)
    task_utils.add_object_id_argument(task, object_id, StringList)

    # move stuff over 24 hours old
    task_utils.add_duration_argument(task, rospy.Duration(60 * 60 *24))
    
    # and delete afterwards
    task_utils.add_bool_argument(task, delete_after_move)
    return task



class MarathonRoutine(PatrolRoutine):
    """ Creates a routine that mixes specific tasks with patrolling nodes."""

    def __init__(self, daily_start, daily_end, tour_duration_estimate=None, idle_duration=rospy.Duration(5), charging_point = 'ChargingPoint'):
        super(MarathonRoutine, self).__init__(daily_start=daily_start, daily_end=daily_end, tour_duration_estimate=tour_duration_estimate, idle_duration=idle_duration, charging_point=charging_point)        

    def create_3d_scan_routine(self, waypoints=None, daily_start=None, daily_end=None, repeat_delta=None):
        """
                    If waypoints now supplied, use all waypoints.

        """
        if waypoints is None: 
            waypoints = self.get_nodes()
        tasks = [ create_3d_scan_task(n) for n in waypoints ]
        self.create_task_routine(tasks=tasks, daily_start=daily_start, daily_end=daily_end, repeat_delta=repeat_delta)


    def create_simple_follow_routine(self, waypoints=None, duration=rospy.Duration(300), daily_start=None, daily_end=None, repeat_delta=None):
        if waypoints is None: 
            waypoints = self.get_nodes()
        tasks = [ create_simple_follow_task(n, duration=duration) for n in waypoints ]
        self.create_task_routine(tasks=tasks, daily_start=daily_start, daily_end=daily_end, repeat_delta=repeat_delta)

    def create_rgbd_record_routine(self, waypoints=None, duration=rospy.Duration(30), camera='head_xtion', with_depth=True, with_rgb=True, daily_start=None, daily_end=None, repeat_delta=None):
        if waypoints is None: 
            waypoints = self.get_nodes()
        tasks = [ create_rgbd_record_task(n, duration=duration, camera=camera, with_depth=with_depth, with_rgb=with_rgb) for n in waypoints ]
        self.create_task_routine(tasks=tasks, daily_start=daily_start, daily_end=daily_end, repeat_delta=repeat_delta)


    def create_tweet_routine(self, twitter_waypoints, img_topic='/head_xtion/rgb/image_color', daily_start=None, daily_end=None, repeat_delta=None):
        """
            [[waypoint, tweet][waypoint, tweet]]
        """
        tasks = [ create_tweet_task(w, t, img_topic) for [w, t] in twitter_waypoints ]
        self.create_task_routine(tasks=tasks, daily_start=daily_start, daily_end=daily_end, repeat_delta=repeat_delta)

    def message_store_entries_to_replicate(self, collections, db='message_store', delete_after_move=True):
        # this is how you add something for when the robot is charging, but these tasks aren't allowed a location
        mongodb_task = create_mongodb_store_task(db, collections, delete_after_move)
        self.add_night_task(mongodb_task)


    def on_idle(self):
        # generate a random waypoint visit on idle
        PatrolRoutine.on_idle(self)    
        # pass
