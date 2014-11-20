#!/usr/bin/env python

import rospy

from dateutil.tz import tzutc, tzlocal
from datetime import time

from routine_behaviours.marathon_routine import MarathonRoutine
    

if __name__ == '__main__':
    rospy.init_node("marathon_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    localtz = tzutc()
    start = time(8,30, tzinfo=localtz)
    end = time(0,37, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    # how long do you want it to take to do a tour. this must be greater than the time you think it will take!
    # the number argument is in seconds
    # tour_duration_estimate = rospy.Duration(60 * 40 * 2)

    # for dummy testing
    tour_duration_estimate = rospy.Duration(60)


    print start
    routine = MarathonRoutine(daily_start=start, daily_end=end, 
        idle_duration=idle_duration, tour_duration_estimate=tour_duration_estimate)    



    scan_waypoints = ['WayPoint2', 'WayPoint3']
    routine.create_3d_scan_routine(waypoints=scan_waypoints)
    
    routine.start_routine()

    rospy.spin()
