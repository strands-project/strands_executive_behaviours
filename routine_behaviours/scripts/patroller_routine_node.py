#!/usr/bin/env python

import rospy

from datetime import time, date
from dateutil.tz import tzlocal

from routine_behaviours.patrol_routine import PatrolRoutine
    

if __name__ == '__main__':
    rospy.init_node("patroller_routine")

    if rospy.get_param('use_sim_time', False):
        from rosgraph_msgs.msg import Clock 
        rospy.loginfo('Using sim time, waiting for time update')
        rospy.wait_for_message('/clock', Clock)


    charging_points = rospy.get_param("~charging_points", ['ChargingPoint1', 'ChargingPoint2'])
    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(0,01, tzinfo=localtz)
    end = time(23,59, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)
    print "AHAHAH", charging_points
    
    routine = PatrolRoutine(charging_point = charging_points, daily_start=start, daily_end=end, idle_duration=idle_duration)    
     
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
