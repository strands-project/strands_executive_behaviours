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
    end = time(0,37, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    # how long you think it will take to do a complete tour. overestimate is better than under
    # tour_duration_estimate = rospy.Duration(60 * 40 * 2)

# for dummy testing
    tour_duration_estimate = rospy.Duration(60)

    routine = PatrolRoutine(daily_start=start, daily_end=end, 
        idle_duration=idle_duration, tour_duration_estimate=tour_duration_estimate)    
    
    # set tasks and start execution
    routine.create_routine()
    routine.start_routine()
    
    rospy.spin()
