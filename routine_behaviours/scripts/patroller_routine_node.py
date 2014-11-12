#!/usr/bin/env python

import rospy

from datetime import time, date
from dateutil.tz import tzlocal

from routine_behaviours.patrol_routine import PatrolRoutine
    

if __name__ == '__main__':
    rospy.init_node("patroller_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,45, tzinfo=localtz)
    end = time(23,45, tzinfo=localtz)

    # create the routine object
    routine = PatrolRoutine(start, end)    
    
    # set tasks and start execution
    routine.create_routine()

    rospy.spin()
