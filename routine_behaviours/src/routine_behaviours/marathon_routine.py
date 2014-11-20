#!/usr/bin/env python

import rospy

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store_msgs.msg import StringList
from mongodb_store.message_store import MessageStoreProxy

from dateutil.tz import tzlocal

from datetime import datetime, timedelta, date, time


from task_executor.task_routine import delta_between
from routine_behaviours.patrol_routine import PatrolRoutine, create_patrol_task

import random


def create_3d_scan_task(waypoint_name):
    task = Task(start_node_id=waypoint_name, end_node_id=waypoint_name, action='ptu_pan_tilt_metric_map', max_duration=rospy.Duration(240))
    task_utils.add_int_argument(task, '-150')
    task_utils.add_int_argument(task, '60')
    task_utils.add_int_argument(task, '160')
    task_utils.add_int_argument(task, '-30')
    task_utils.add_int_argument(task, '20')
    task_utils.add_int_argument(task, '30')
    return task




class MarathonRoutine(PatrolRoutine):
    """ Creates a routine that mixes specific tasks with patrolling nodes."""

    def __init__(self, daily_start, daily_end, tour_duration_estimate=None, idle_duration=rospy.Duration(5)):
        super(MarathonRoutine, self).__init__(daily_start=daily_start, daily_end=daily_end, tour_duration_estimate=tour_duration_estimate, idle_duration=idle_duration)        


    def create_task_routine(self, task_creation_fn, waypoints=None, daily_start=None, daily_end=None, repeat_delta=timedelta(hours=1)):

        if not waypoints:
            waypoints = self.get_nodes()

        if not daily_start:
            daily_start = self.daily_start

        if not daily_end:
            daily_end = self.daily_end

        print daily_start
        print daily_end

        tasks = [ task_creation_fn(n) for n in waypoints ]

        self.routine.repeat_every_delta(tasks, delta=repeat_delta, times=1, start_time=daily_start, duration=delta_between(daily_start, daily_end))


    def create_3d_scan_routine(self, waypoints=None, daily_start=None, daily_end=None, repeat_delta=timedelta(hours=1)):
        self.create_task_routine(task_creation_fn=create_patrol_task, waypoints=waypoints, daily_start=daily_start, daily_end=daily_end, repeat_delta=repeat_delta)

    def create_patrol_routine(self):

        # create the patrolling elements
        PatrolRoutine.create_routine(self)

        # localtz = tzlocal()

        # self.runner.add_day_off('Saturday')
        # self.runner.add_day_off('Sunday')        
        # self.runner.add_date_off(date(2014, 05, 26))

        # NIGHT TASKS
        # this is how you add something for when the robot is charging, but these tasks aren't allowed a location
        # clear_datacentre_task = create_datacentre_task(['heads','metric_map_data','rosout_agg','robot_pose','task_events','scheduling_problems','ws_observations','monitored_nav_events'])
        # self.add_night_task(clear_datacentre_task)


    def on_idle(self):
        # generate a random waypoint visit on idle
        PatrolRoutine.on_idle(self)    

