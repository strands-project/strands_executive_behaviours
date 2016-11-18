#!/usr/bin/env python

from __future__ import with_statement 

import rospy
import threading

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task, ExecutionStatus
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus, DemandTask, GetIDs
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from mongodb_store.message_store import MessageStoreProxy

from scitos_msgs.msg import BatteryState

from dateutil.tz import *
from datetime import *

from task_executor import task_routine
from task_executor.utils import get_start_node_ids
from task_executor.task_routine import delta_between
from copy import deepcopy

from dynamic_reconfigure.server import Server
from routine_behaviours.cfg import RoutineParametersConfig

NO_THRESHOLD = 2

UNDER_SOFT_THRESHOLD = 1

UNDER_HARD_THRESHOLD = 0


class RobotRoutine(object):
    """ This class manages the over-arching routine behaviour of the robot. It monitors battery charge task level and can force charging when necessary and also generate tasks when the robot is idle. """

    def __init__(self, daily_start, daily_end, idle_duration, charging_point = 'ChargingPoint', pre_start_window=timedelta(hours=1)):
        """
        Args:
            daily_start (datetime.time): The time of day the routine should start.
            daily_end (datetime.time): The time of day the routine should end.
            idle_duration (rospy.Duration): How long to be idle before triggering on_idle.
            charging_point: Where to head to when charge is low
        """

        self._schedule_lock = threading.Lock()

        self.daily_start = daily_start
        self.daily_end = daily_end
        if not isinstance(charging_point, list):
            charging_point = [charging_point]
        self._charging_points = charging_point
        self.pre_start_window = pre_start_window
        self._create_services()
                
        self._routine_is_paused = False

        rospy.loginfo('Fetching parameters from dynamic_reconfigure')
        Server(RoutineParametersConfig, self.dynamic_reconfigure_cb)


        self.battery_count = 0
        # home many ~10Hz updates to wait for between forced charge counts
        self.battery_count_thres = 10 * 60 * 5
        self.battery_state = None

        self.battery_threshold_state = NO_THRESHOLD
        self.battery_threshold_state_change = 0


        # how long to charge for when force_charge_threshold is triggered
        self.force_charge_duration = rospy.Duration(2 * 60)
        self.sent_night_tasks = False
        self.night_tasks = []

        rospy.Subscriber('/battery_state', BatteryState, self._check_battery)
        # lazy/silly sleep to check battery received if present
        rospy.sleep(0.5)

        
        # create routine structure
        self.routine = self.new_routine()

        # create the object which will talk to the scheduler
        self.runner = self.new_routine_runner()


        # calculate how long to sleep for overnight
        start_time_secs = task_routine.time_to_secs(daily_start)
        end_time_secs = task_routine.time_to_secs(daily_end)
        full_day_secs = task_routine.time_to_secs(time(23,59))

        # this is the whole night
        night_secs = (full_day_secs - end_time_secs) + start_time_secs
        self.night_duration = rospy.Duration(night_secs)

        # how big a gap in the scheduler should trigger a return to charger 
        self.charge_window = rospy.Duration(60 * 10)

        # and how long to charge for once there, it won't automatically leave if there's nothing to do
        self.idle_charge_duration = rospy.Duration(60)

        self.idle_count = 0

        idle_interval_check = rospy.Duration(5)
        self._idle_timer = rospy.Timer(period = idle_interval_check, callback = self._check_idle)

        # how many idle counts to wait for before deciding we're idle
        self.idle_thres = int(idle_duration.to_sec() / idle_interval_check.to_sec())

        self.sent_to_charge = False

         # Set the task executor running in case it's not
        self.set_execution_status(True)

        self.schedule = None
        rospy.Subscriber('current_schedule', ExecutionStatus, self._save_schedule)

        self._current_node = None
        rospy.Subscriber('current_node', String, self._update_topological_location)

        # allow other clients to queue up tasks for 
        rospy.Service('robot_routine/add_tasks', AddTasks, self._add_new_tasks_to_routine_srv)

        # allow 
        rospy.Service('robot_routine/pause_routine', Empty, self._pause_routine)
        rospy.Service('robot_routine/unpause_routine', Empty, self._unpause_routine)

    def new_routine(self):
        return task_routine.DailyRoutine(self.daily_start, self.daily_end)

    def new_routine_runner(self):
        return task_routine.DailyRoutineRunner(self.daily_start, self.daily_end, self.add_tasks, day_start_cb=self.on_day_start, day_end_cb=self.on_day_end, tasks_allowed_fn=self.task_allowed_now, daily_tasks_fn=self.extra_tasks_for_today, pre_start_window=self.pre_start_window)


    def _pause_routine(self, req):
        rospy.loginfo('Pausing routine')
        self._routine_is_paused = True
        return EmptyResponse()

    def _unpause_routine(self, req):
        rospy.loginfo('Unpausing routine')
        self._routine_is_paused = False
        return EmptyResponse()

    def _update_topological_location(self, node_name):
        self._current_node = node_name.data

    def extra_tasks_for_today(self):
        """
        Return a list of extra tasks for the day ahead. Called every morning before the routine day starts.
        """
        rospy.loginfo('extra_tasks_for_today')
        return []


    def _is_time_critical(self, task):
        return task.start_after == task.end_before

    def task_allowed_now(self, task):
        """ 
        Return true if a task can be send to the task_executor now.


        This checks if battery level is fine and that the robot is not at the charging point. Subclasses can override this to provide additional checks.

        """
        # if routine is paused then no tasks allowed
        if self._routine_is_paused:
            return False

        for wp in get_start_node_ids(task):
            if wp in self.blacklisted_nodes:
                rospy.logwarn('Node was blacklisted for task: %s' % wp)
                return False
            
        # if battery is above soft threshold or has charged enough 
        if self.battery_ok():
            rospy.loginfo('Battery is ok for task')
            return True
            
        # else we're about the hard threshold and the task is time critical
        if self.battery_state.lifePercent > self.threshold and self._is_time_critical(task):
            return True
        
        return (task.start_node_id in self._charging_points and self._current_node == task.start_node_id)

    def dynamic_reconfigure_cb(self, config, level):
        conf = self.battery_thresholds_cb(config, level)
        conf = self.blacklisted_nodes_cb(config, level)
        return conf

    def battery_thresholds_cb(self, config, level):
        rospy.loginfo("Battery thresholds set to {force_charge_threshold}, {force_charge_addition}, {soft_charge_threshold}".format(**config))

        self.threshold = config['force_charge_threshold']
        self.addition = config['force_charge_addition']
        self.soft_threshold = config['soft_charge_threshold']

        if self.soft_threshold < self.threshold:
            rospy.logwarn('soft_threshold less than hard threshold, updating')
            self.soft_threshold = self.threshold

        return config

    def blacklisted_nodes_cb(self, config, level):


        self.blacklisted_nodes = map(str.strip, config['blacklisted_nodes'].split(','))        

        if len(self.blacklisted_nodes) and self.blacklisted_nodes[0] == '':
            self.blacklisted_nodes = []

        rospy.loginfo("Blacklisted nodes set to: %s" % self.blacklisted_nodes)
        return config


    def add_night_task(self, task):
        """
        Add a task to be executed after the routine ends. These tasks cannot involve movement and therefore must either have an empty start_node_id or be performed at the charging station.
        """
        if task.start_node_id != ''  and not task.start_node_id in self._charging_points:
            rospy.logwarn('Rejecting task night task as only self._charging_points tasks are allowed at night')
            return 

        self.night_tasks.append(task)

    def is_before_day_start(self,time):
        """
        Check if the given time is before the start of the routine.
        """
        if task_routine.time_less_than(self.daily_start,self.daily_end):  
            return task_routine.time_less_than(time, self.daily_start)
        else:
            return task_routine.time_less_than(time, self.daily_start) and task_routine.time_less_than(self.daily_end, time)

    def is_after_day_end(self,time):
        """
        Check if the given time is after the end of the routine.
        """        
        if task_routine.time_less_than(self.daily_start,self.daily_end):  
            return task_routine.time_less_than(self.daily_end, time)
        else:
            return task_routine.time_less_than(self.daily_end, time) and task_routine.time_less_than(time, self.daily_start)

    def is_during_day(self,time):
        """
        Check if the given time is during the routine.
        """        
        return not (self.is_before_day_start(time) or self.is_after_day_end(time))

    def _save_schedule(self, schedule):
        with self._schedule_lock:
            self.schedule = schedule
            rospy.loginfo('updated schedule')



    def _check_idle(self, time_event):
        with self._schedule_lock:
            if self.schedule is None:
                return

            # only check within working hours
            rostime_now = rospy.get_rostime()
            now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal()).time()

            # if the task to get the robot on to the charge station fails is some way it could 
            # get strandard. The battery recovery will kick in later, but we need to keep trying 
            # in order to get the night tasks launched, which are only sent when charging


            # on_charger = self.battery_state is not None and (self.battery_state.charging or self.battery_state.powerSupplyPresent)

            # now that topological navigation supports localisation by topic, 
            # we can assume that if our location is the charging point then
            # we are also charging
            on_charger = (self._current_node in self._charging_points)


            # if it's night time, we're not doing anything and we're not on the charger
            if not self.is_during_day(now) and not self.schedule.currently_executing and not on_charger:
                self.demand_charge(rospy.Duration(10 * 60.0))

            elif self.is_during_day(now) and self.battery_ok() :

                if self.schedule.currently_executing:
                    self.idle_count = 0
                    return
                elif len(self.schedule.execution_queue) == 0:
                    self.idle_count += 1
                else:
                    # delay until next task
                    delay_until_next = self.schedule.execution_queue[0].execution_time - rostime_now
                    # rospy.loginfo('delay until next: %s' % delay_until_next.to_sec())
                    
                    
                    if delay_until_next > rospy.Duration(60):
                        self.idle_count += 1
                    else:
                       self.idle_count = 0

            else:
                self.idle_count = 0

            # rospy.loginfo('idle count: %s' % self.idle_count)
            # rospy.loginfo('idle threshold: %s' % self.idle_thres)

            if self.idle_count > self.idle_thres:
                if not self.runner.day_off() and not self._routine_is_paused:
                    self.on_idle()
                self.idle_count = 0


    def add_new_tasks_to_routine(self, tasks):
        self.runner.insert_extra_tasks(tasks) 

    def _add_new_tasks_to_routine_srv(self, req):
        task_ids = self.new_task_ids(len(req.tasks)).task_ids
        for idx, task in enumerate(req.tasks):
            task.task_id = task_ids[idx]

        self.add_new_tasks_to_routine(req.tasks) 
        return [task_ids]

    def battery_ok(self):
        """ Reports false if battery is below force_charge_threshold or if it is above it but within force_charge_addition of the threshold and charging """ 

        if self.battery_state is not None:

            # if batter is below threshold, it's never ok
            if self.battery_state.lifePercent < self.soft_threshold:                
                return False
            # else if we're charging we should allow some amount of charging to happen
            # before everything is ok again
            elif self.battery_state.charging or self.battery_state.powerSupplyPresent:
                threshold = min(self.soft_threshold + self.addition, 98)                
                return self.battery_state.lifePercent > threshold
            else:
                return True

        else: 
            rospy.logwarn('No battery received when checking')
            return True

    def _update_battery_threshold(self):
        """
        Updates the discrete battery threshold of the routine.
        """

        if self.battery_state.lifePercent < self.threshold:
            new_state = UNDER_HARD_THRESHOLD
        elif  self.battery_state.lifePercent < self.soft_threshold:
            new_state = UNDER_SOFT_THRESHOLD
        else:
            new_state = NO_THRESHOLD

        self.battery_threshold_state_change = new_state - self.battery_threshold_state
        self.battery_threshold_state = new_state


    def _check_battery(self, battery_state):
        self.battery_state = battery_state

        self._update_battery_threshold()

        if self.battery_threshold_state_change < 0:
            rospy.logwarn('Passed into a lower battery threshold, clearing execution schedule')
            self.clear_execution_schedule()

        if not self.battery_ok():
            
            # if not ok and not triggered yet
            if self.battery_count == 0:
                # if we're below soft threshold but above hard threshold then add a charge tasks not clear
                if self.battery_state.lifePercent > self.threshold:
                    rospy.logwarn('Battery below soft charge threshold: %s ' % (self.battery_state.lifePercent)) 
                    self.add_charge(self.force_charge_duration)
                    # bit of a hack to delay the update for this to happen again
                    self.battery_count = -(self.battery_count_thres * 2) 

                else:
                    rospy.logwarn('Battery below force charge threshold: %s ' % (self.battery_state.lifePercent))
                    self.clear_then_charge(self.force_charge_duration)
            
            # update count
            self.battery_count += 1
            
            if self.battery_count >= self.battery_count_thres and not (self.battery_state.charging or self.battery_state.powerSupplyPresent):                
                rospy.logwarn('Still not charging, trying again')
                self.battery_count = 0
        else:
            self.battery_count = 0

        # check for charging so we can send off night tasks
        rostime_now = rospy.get_rostime()
        now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal()).time()

        if len(self.night_tasks) > 0 and not self.sent_night_tasks and self.battery_state is not None and self.battery_state.charging and not self.is_during_day(now):
            rospy.loginfo('Sending night tasks')
            self._send_night_tasks()
         
    def _send_night_tasks(self):
        instantiated_night_tasks = []

        now = rospy.get_rostime()
    
        # hack to push scheduler to do them in the order they were added
        delta = rospy.Duration(600)

        for task in self.night_tasks:
            night_task = deepcopy(task)
            night_task.start_after = now
            # some arbitraty time  -- 12 hours -- in the future
            night_task.end_before = now + rospy.Duration(60 * 60 * 12)
            instantiated_night_tasks.append(night_task)
            now = now + delta

        try:
            self.add_tasks(instantiated_night_tasks)
            self.sent_night_tasks = True
        except Exception, e:
            rospy.logwarn('Exception on scheduler service call: %s' % e)


    def _create_services(self):        
        self.maps_msg_store = MessageStoreProxy(collection='topological_maps')


    def clear_schedule(self):
        try:
            clear_schedule_srv_name = '/task_executor/clear_schedule'
            rospy.wait_for_service(clear_schedule_srv)
            clear_schedule_srv = rospy.ServiceProxy(clear_schedule_srv_name, Empty)
            clear_schedule_srv()
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)
    

    def new_task_ids(self, count):
        try:
            task_ids_srv_name = '/task_executor/get_ids'        
            rospy.wait_for_service(task_ids_srv_name)
            new_task_ids_srv = rospy.ServiceProxy(task_ids_srv_name, GetIDs)
            return new_task_ids_srv(count)
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)


    def demand_task(self, task):
        try:
            demand_task_srv_name = '/task_executor/demand_task'
            rospy.wait_for_service(demand_task_srv)
            demand_task_srv = rospy.ServiceProxy(demand_task_srv_name, DemandTask)
            demand_task_srv(task)
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)


    def add_tasks(self, tasks):
        try:            
            add_tasks_srv_name = '/task_executor/add_tasks'
            rospy.wait_for_service(add_tasks_srv_name)
            add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)
            # ensure this is always done in case of failures
            self.set_execution_status(True)
            return add_tasks_srv(tasks)
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)
            return []


    def set_execution_status(self, status):
        try:                        
            set_exe_stat_srv_name = '/task_executor/set_execution_status'
            rospy.wait_for_service(set_exe_stat_srv_name)                    
            set_execution_status_srv = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)        
            set_execution_status_srv(status)
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)


    def _create_charge_task(self, charge_duration):
        charge_task = Task(action='wait_action', max_duration=charge_duration)
        charge_task.start_node_id = ''
        charge_task.priority = Task.HIGH_PRIORITY

        # allow it to charge at any of the points
        for wp in self._charging_points:
            charge_task.start_node_id += '%s | ' % wp
        charge_task.start_node_id = charge_task.start_node_id[:-3] 

        charge_task.start_after = rospy.get_rostime()
        charge_task.end_before = charge_task.start_after + rospy.Duration(60 * 60)

        task_utils.add_time_argument(charge_task, rospy.Time())
        task_utils.add_duration_argument(charge_task, charge_duration)       
        return charge_task




    def demand_charge(self, charge_duration):
        """
        Create an on-demand task to charge the robot for the given duration.
        """
        try:
            self.demand_task(self._create_charge_task(charge_duration))
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)

    def add_charge(self, charge_duration):
        """
        Create an on-demand task to charge the robot for the given duration.
        """
        try:
            self.add_tasks([self._create_charge_task(charge_duration)])
        except rospy.ServiceException, e:
            rospy.logwarn('Service threw exception: %s'% e)


    def clear_execution_schedule(self):
        """
        Clears execution schedule of the robot.
        """
        try:
            self.clear_schedule()
        except rospy.ServiceException, e:
            rospy.logwarn('Empty service complaint occurs here. Should be safe: %s'% e)


    def clear_then_charge(self, charge_duration):
        """ Clears the schedule of the robot before demanding a charge task. """

        self.clear_execution_schedule()

        # safety sleep, but could be issues here
        rospy.sleep(10)
        self.demand_charge(charge_duration)

    def on_day_end(self):
        """
        Triggered when the routine ends for the day.

        This clears tasks and triggers an on-demand charge.
        """
        # end of day, charge for 10 mins; will charge all night as no moves allowed.
        self.clear_then_charge( rospy.Duration(10 * 60.0) ) 

        rospy.loginfo('ok, I\'m calling it a day until %s' % datetime.fromtimestamp((rospy.get_rostime() + self.night_duration).to_sec()))

       
    def on_day_start(self):        
        """
        Triggered when the routine starts for the day.
        """
        rospy.loginfo('Good morning')
        self.sent_night_tasks = False

    def start_routine(self):
        """
        Starts the routine running. Must be called to generate behaviour.
        """
        self.runner.add_tasks(self.routine.get_routine_tasks())

    def on_idle(self):
        """
            Called when the routine is idle. Default is to trigger travel to the charging. As idleness is determined by the current schedule, if this call doesn't utlimately cause a task schedule to be generated this will be called repeatedly.
        """
        rospy.loginfo('Idle for too long, going to charging point')
        # go for a quick charge
        self.demand_charge(self.idle_charge_duration)

    def create_task_routine(self, tasks, daily_start=None, daily_end=None, repeat_delta=None):
        """ 
            Create routine behaviour from given tasks using provided parameter. 

            This largely passes arguments on to similar arguments in task_routine.repeat_every_delta, but constrains repeats to the bounds of the this routine.

            If daily start or end not supplied use routine start or end
            If delta not supplied, just do once during the start to end window
        """

        if daily_start is None:
            daily_start = self.daily_start

        if daily_end is None:
            daily_end = self.daily_end

        if repeat_delta is None:
            repeat_delta = delta_between(daily_start, daily_end)


        self.routine.repeat_every_delta(tasks, delta=repeat_delta, times=1, start_time=daily_start, duration=delta_between(daily_start, daily_end))



 
