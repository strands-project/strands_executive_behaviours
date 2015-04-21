#!/usr/bin/env python

import rospy

from datetime import time, date
from dateutil.tz import tzlocal

from routine_behaviours.patrol_routine import PatrolRoutine
    

if __name__ == '__main__':
    rospy.init_node("patroller_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,30, tzinfo=localtz)
    end = time(17,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    routine = PatrolRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)    
     
    # create a routine to patrol all points in the environment
    # 
    # if you want to patrol different points use create_patrol_routine instead of
    # create_routine and you can use the all_waypoints and all_waypoints_except 
    # methods to configure locations to patrol
    # 
    routine.create_routine()
 
    # set tasks and start execution
    routine.start_routine()

    rospy.spin()
