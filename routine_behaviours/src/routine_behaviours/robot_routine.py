#!/usr/bin/env python

import rospy

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task, ExecutionStatus
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus, DemandTask
from std_srvs.srv import Empty
from mongodb_store.message_store import MessageStoreProxy

from scitos_msgs.msg import BatteryState

from dateutil.tz import *
from datetime import *

from task_executor import task_routine
from task_executor.task_routine import delta_between
from copy import deepcopy

from dynamic_reconfigure.server import Server
from routine_behaviours.cfg import ChargingThresholdsConfig


class RobotRoutine(object):

    """Wraps up all the routine stuff with charging etc."""
    def __init__(self, daily_start, daily_end, idle_duration):

        self.daily_start = daily_start
        self.daily_end = daily_end
        self._create_services()

        rospy.loginfo('Fetching parameters from dynamic_reconfigure')
        self.recfg_sever = Server(ChargingThresholdsConfig, self.dynamic_reconfigure_cb)
        

        self.battery_count = 0
        # home many ~10Hz updates to wait for between forced charge counts
        self.battery_count_thres = 10 * 60 * 5
        self.battery_state = None
        # how long to charge for when force_charge_threshold is triggered
        self.force_charge_duration = rospy.Duration(60 * 60 * 2)
        self.sent_night_tasks = False
        self.night_tasks = []

        rospy.Subscriber('/battery_state', BatteryState, self._check_battery)
        # lazy/silly sleep to check battery received if present
        rospy.sleep(0.5)

        

        # create routine structure
        self.routine = task_routine.DailyRoutine(daily_start, daily_end)
        # create the object which will talk to the scheduler
        self.runner = task_routine.DailyRoutineRunner(self.daily_start, self.daily_end, self.add_tasks, day_start_cb=self.on_day_start, day_end_cb=self.on_day_end, tasks_allowed_fn=self.battery_ok)


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

        # how many 5 second idle counts to wait for before deciding we're idle
        # a count of 12 is one minute
        


        self.idle_thres = int(idle_duration.to_sec() / 5)

        self.sent_to_charge = False

         # Set the task executor running in case it's not
        self.set_execution_status(True)

        rospy.Subscriber('current_schedule', ExecutionStatus, self._check_idle)

    def dynamic_reconfigure_cb(self, config, level):
        rospy.loginfo("Config set to {force_charge_threshold}, {force_charge_addition}".format(**config))

        self.threshold = config['force_charge_threshold']
        self.addition = config['force_charge_addition']
        return config

    def add_night_task(self, task):
        if task.start_node_id == 'ChargingPoint':
            task.start_node_id = ''

        if task.start_node_id != '':
            rospy.logwarn('Rejecting task to do %s at %s as only location-free tasks are allowed at night')
            return 

        self.night_tasks.append(task)

    def is_before_day_start(self,time):
        if task_routine.time_less_than(self.daily_start,self.daily_end):  
            return task_routine.time_less_than(time, self.daily_start)
        else:
            return task_routine.time_less_than(time, self.daily_start) and task_routine.time_less_than(self.daily_end, time)

    def is_after_day_end(self,time):
        if task_routine.time_less_than(self.daily_start,self.daily_end):  
            return task_routine.time_less_than(self.daily_end, time)
        else:
            return task_routine.time_less_than(self.daily_end, time) and task_routine.time_less_than(time, self.daily_start)

    def is_during_day(self,time):
        return not (self.is_before_day_start(time) or self.is_after_day_end(time))


    def _check_idle(self, schedule):

        # only check within working hours
        rostime_now = rospy.get_rostime()
        now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal()).time()

        if self.battery_ok() and self.is_during_day(now):

            if schedule.currently_executing:
                self.idle_count = 0
                return
            if len(schedule.execution_queue) == 0:
                self.idle_count += 1
            else:
                # delay until next task
                delay_until_next = schedule.execution_queue[0].execution_time - rostime_now
                # rospy.loginfo('delay until next: %s' % delay_until_next.to_sec())
                if delay_until_next > self.charge_window:
                    self.idle_count += 1
                else:
                   self.idle_count = 0

        else:
            self.idle_count = 0

        # rospy.loginfo('idle count: %s' % self.idle_count)
        # rospy.loginfo('idle threshold: %s' % self.idle_thres)

        if self.idle_count > self.idle_thres:
            self.on_idle()


    def battery_ok(self):
        """ Reports false if battery is below force_charge_threshold or if it is above it but within force_charge_addition of the threshold and charging """ 

        if self.battery_state is not None:

            # if batter is below threshold, it's never ok
            if self.battery_state.lifePercent < self.threshold:                
                return False
            # else if we're charging we should allow some amount of charging to happen
            # before everything is ok again
            elif self.battery_state.charging or self.battery_state.powerSupplyPresent:
                threshold = min(self.threshold + self.addition, 99)                
                return self.battery_state.lifePercent > threshold
            else:
                return True

        else: 
            rospy.logwarn('No battery received when checking')
            return True

    def _check_battery(self, battery_state):
        self.battery_state = battery_state


        if not self.battery_ok():
            
            # if not ok and not triggered yet
            if self.battery_count == 0:
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
            self.send_night_tasks()
         
    def send_night_tasks(self):
        instantiated_night_tasks = []

        now = rospy.get_rostime()
    
        for task in self.night_tasks:
            night_task = deepcopy(task)
            night_task.start_after = now
            # some arbitraty time  -- 6 hours -- in the future
            night_task.end_before = now + rospy.Duration(60 * 60 * 12)
            instantiated_night_tasks.append(night_task)


        self.add_tasks(instantiated_night_tasks)
        self.sent_night_tasks = True

    def _create_services(self):
        add_tasks_srv_name = '/task_executor/add_tasks'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        demand_task_srv_name = '/task_executor/demand_task'
        clear_schedule_srv_name = '/task_executor/clear_schedule'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_tasks_srv_name)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Done")        
        self.add_tasks = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)
        self.set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        self.demand_task = rospy.ServiceProxy(demand_task_srv_name, DemandTask)
        self.clear_schedule = rospy.ServiceProxy(clear_schedule_srv_name, Empty)
        self.maps_msg_store = MessageStoreProxy(collection='topological_maps')


    # deprecated
    # def load_nodes(self):
    #     msg_store = MessageStoreProxy(collection='topological_maps')
    #     query_meta = {}
    #     query_meta["pointset"] = rospy.get_param('topological_map_name')
    #     nodes = self.maps_msg_store.query(TopologicalNode._type, {}, query_meta)
    #     return [n for [n, meta] in nodes]

    def demand_charge(self, charge_duration):
        charging_point = 'ChargingPoint'
        charge_task = Task(action='wait_action', start_node_id=charging_point, end_node_id=charging_point, max_duration=charge_duration)
        task_utils.add_time_argument(charge_task, rospy.Time())
        task_utils.add_duration_argument(charge_task, charge_duration)       
        self.demand_task(charge_task)

    def clear_then_charge(self, charge_duration):
        try:
            self.clear_schedule()
        except rospy.ServiceException, e:
            rospy.loginfo('Empty service complaint occurs here. Should be safe: %s'% e)

        # safety sleep, but could be issues here
        rospy.sleep(10)
        self.demand_charge(charge_duration)

    def on_day_end(self):
        # end of day, charge for 10 mins; will charge all night as no moves allowed.
        self.clear_then_charge( rospy.Duration(10 * 60.0) ) 

        rospy.loginfo('ok, I\'m calling it a day until %s' % datetime.fromtimestamp((rospy.get_rostime() + self.night_duration).to_sec()))

       
    def on_day_start(self):        
        rospy.loginfo('Good morning')
        self.sent_night_tasks = False

    def start_routine(self):
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
            If daily start or end not supplied use routine start or end
            If delta not supplied, just do once during the start to end window
        """

        if not daily_start:
            daily_start = self.daily_start

        if not daily_end:
            daily_end = self.daily_end

        if not repeat_delta:
            repeat_delta = delta_between(daily_start, daily_end)


        self.routine.repeat_every_delta(tasks, delta=repeat_delta, times=1, start_time=daily_start, duration=delta_between(daily_start, daily_end))



