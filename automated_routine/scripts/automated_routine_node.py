#!/usr/bin/env python
import rospy
import sys

from datetime import time, date, timedelta, datetime
from dateutil.tz import tzlocal
import dynamic_reconfigure.client
import actionlib

from routine_behaviours.patrol_routine import PatrolRoutine
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
#from strands_gazing.msg import GazeAtPoseAction, GazeAtPoseGoal

from task_executor.utils import rostime_to_python
import dynamic_reconfigure.client

class AutomatedRoutine(PatrolRoutine):

    def __init__(self, daily_start, daily_end, idle_duration=rospy.Duration(5 * 60), charging_point='ChargingPoint', pre_start_window=timedelta(hours=1)):

        super(TSCRoutine, self).__init__(daily_start=daily_start, daily_end=daily_end, idle_duration=idle_duration, charging_point=charging_point, pre_start_window=pre_start_window)

        if isinstance(charging_point, list):
            self.random_nodes = charging_point
        else:
            self.random_nodes = [charging_point]

    def start_routine(self):
        self.runner.add_tasks(self.routine.get_routine_tasks())

    def extra_tasks_for_today(self):
        """
        Return a list of extra tasks for the day ahead. Called every morning before the routine day starts.
        """

        return []

    def on_idle(self):
        rospy.loginfo('Idle for too long, going to charging point')
        # go for a quick charge
        charge_task = self._create_charge_task(self.idle_charge_duration)
        charge_task.priority = Task.LOW_PRIORITY
        charge_task.end_before = charge_task.start_after + rospy.Duration(60 * 5) + self.idle_charge_duration
        self.add_new_tasks_to_routine([charge_task])

def run_battery_levels(battery_levels, tz = None):
    """Checks to ensure that battery levels remain in the correct range, and
    modifies the levels if the configuration requires them to be different at
    different times of day

    """
    if tz is None:
        tz = tzlocal()

    # process the bare times to be datetimes
    now = rostime_to_python(rospy.get_rostime(), tz = tz)

    for t in battery_levels:
        t[0] = now.replace(hour=t[0].hour, minute=t[0].minute)
        print t[0]

    while not rospy.is_shutdown():

        changed_interval = False

        # current time as a python datetime
        now = rostime_to_python(rospy.get_rostime(), tz = tz)

        print 'Seeing whether I need to reconfigure battery thresholds'

        while not (now > battery_levels[0][0] and now < battery_levels[1][0]):
            for_tomorrow = battery_levels.pop(0)
            for_tomorrow[0] += timedelta(days=1)
            battery_levels.append(for_tomorrow)
            changed_interval = True

        if changed_interval:
            rospy.loginfo('Changing battery levels')
            print battery_levels[0]
            try:
                client = dynamic_reconfigure.client.Client(rospy.get_name(), timeout = 5)
                params = { 'force_charge_threshold' : battery_levels[0][1], 'soft_charge_threshold' : battery_levels[0][2] }
                config = client.update_configuration(params)
            except Exception, e:
                print e
                battery_levels.insert(0, battery_levels.pop())


        rospy.sleep(5 * 60)

def get_off_date_list(off_date_conf):
    """Convert the dates off and date ranges off to a list of dates to add to the
    dates off in the routine

    """
    
    dates_off = []
    requested_dates = off_date_conf["dates"]

    for date_string in requested_dates:
        dates_off.append(datetime.datetime.strptime("%Y/%m/%d", date_string).date())

    requested_ranges = off_date_conf["date_ranges"]

    for range_name in requested_ranges:
        start = datetime.datetime.strptime("%Y/%m/%d", requested_ranges[range_name]["start_date"]).date()
        end = datetime.datetime.strptime("%Y/%m/%d", requested_ranges[range_name]["end_date"]).date()

        # swap if dates have been set the wrong way round
        if end < start:
            tmp = start
            start = end
            end = start

        # if the dates are the same just append one date and carry on
        if end == start:
            dates_off.append(start)
            continue

        # At this point it should be a valid date range
        current_off_date = start
        while current_off_date <= end:
            # Args to add_date_off MUST be datetime.date objects!
            dates_off.append(current_off_date)
            rospy.loginfo("Added date {0} to off dates".format(current_off_date))
            current_off_date = current_off_date + timedelta(days=1)

    return dates_off

def get_task_priority(task_config):
    priorities = {"low": Task.LOW_PRIORITY, "medium": Task.MEDIUM_PRIORITY, "high": Task.HIGH_PRIORITY}

    if "priority" in task_config:
        priority = task_config["priority"]
        if isinstance(priority, str):
            if priority in priorities.keys():
                priority = priorities[priority]
            else:
                rospy.logwarn("Unknown priority string {0}. Setting to normal priority".format(priority))
                priority = Task.NORMAL_PRIORITY
    else:
        priority = Task.NORMAL_PRIORITY

    return priority

def waypoints_from_dict(list_key, waypoint_lists):
    """Given the waypoint list you want, and the dict of waypoint lists, return a
    list of waypoints.

    A key in the waypoint_lists dict can have a value of either a list or a
    single string. In the case of a single string, it is assumed to refer to the
    key of another list.

    """

    # while rather than recursion for simplicity
    while isinstance(list_key, str):
        if list_key not in waypoint_lists:
            return []
        list_key = waypoint_lists[list_key]

    return list_key

def get_start_end_waypoints(task_config, config):
    # Waypoints for the task can be either a reference to a waypoint list in
    # the config, or a single waypoint. Both are defined by strings.
    if task_config["waypoints"] in config["waypoint_lists"]:
        waypoints = waypoints_from_dict(task_config["waypoints"], config["waypoint_lists"])
    else:
        waypoints = [task_config["waypoints"]]

    if not any(waypoints):
        rospy.logwarn("Task definition was invalid. Waypoints must be specified and non-null. Got {0}.".format(waypoints))
        return (None, None)

    # Extract the end waypoint for each waypoint, if it exists
    end_waypoints = []
    if "end_waypoints" in task_config:
        if isinstance(task_config["end_waypoints"], list):
            end_waypoints = [None if wp == "none" else wp for wp in task_config["end_waypoints"]]
        elif task_config["end_waypoints"] in config["waypoint_lists"]:
            end_waypoints = config["waypoint_lists"][task_config["end_waypoints"]]
        else:
            end_waypoints = [task_config["end_waypoints"]]
        if len(end_waypoints) != len(waypoints):
            rospy.logwarn("Task definition was invalid. Number of end waypoints must match number of waypoints")
            return (None, None)

    return (waypoints, end_waypoints)

def arg_dict_is_valid(arg_dict):
    # must be a dict and have a type key
    if not isinstance(arg_dict, dict) or "type" not in arg_dict:
        return False

    # if the type is waypoint, does not require a value key
    if arg_dict["type"] != "waypoint" and "value" not in arg_dict:
            return False

    return True

def add_task_arg_from_dict(task, waypoint, arg_dict, task_config, global_config):
    """Add an argument to the given task from data given in the configuration.

    The task_config and global_config dicts are needed as the user can refer to
    dictionaries there to get data

    """
    valid_types = ["string", "time", "duration", "waypoint", "arg", "global"]
    if arg_dict["type"] == "string":
        task_utils.add_string_argument(task, arg_dict["value"])
    elif arg_dict["type"] == "duration":
        task_utils.add_duration_argument(task, int(arg_dict["value"]))
    elif arg_dict["type"] == "time":
        if arg_dict["value"] < 0:
            rospy.logwarn("Time values must be >1.")
            return False
        task_utils.add_time_argument(task, rospy.Time(int(arg_dict["value"])))
    elif arg_dict["type"] == "waypoint":
        task_utils.add_string_argument(task, waypoint)
    elif arg_dict["type"] == "arg":
        if not "args" in task_config:
            rospy.logwarn("Task arg type arg provided, but args does not exist in the task dictionary.")
            return False
        task_utils.add_string_argument(task, task_config["args"][arg_dict["value"]])
    elif arg_dict["type"] == "global":
        if not "global_data" in global_config:
            rospy.logwarn("Task arg type global provided, but global_data does not exist in the task directory")
            return False

        # can use the $waypoint$ string in the dictionary keys to have different
        # values depending on the waypoint
        dict_ref_list = arg_dict["value"].replace("$waypoint$", waypoint).split(":")
        cur_dict = global_config["global_data"]
        # get the data value you want by going through the chain of keys provided
        for ind, key in enumerate(dict_ref_list):
            if key in cur_dict:
                value = cur_dict[key]
                if isinstance(value, dict):
                    cur_dict = value
                else:
                    task_utils.add_string_argument(task, value)
                    if ind < len(dict_ref_list) - 1:
                        rospy.logwarn("Got a value from the dict before processing all keys. Current key is {0}, list is {1}. There might be something wrong with your definition.".format(key, dict_ref_list))
                    break # this should be the last item in the list, but break just in case
            else:
                rospy.logwarn("Key {0} requested from dict, but it doesn't exist".format(key))
                return False

    else:
        rospy.logwarn("Unsupported task_args type \"{0}\". Options are: {1}.".format(arg_dict["type"]), valid_types)
        return False

    return True

def tasks_from_config(task_config, global_config):
    """Convert a dictionary containing data specifying a type of task into a list of
    tasks.

    """
    if "active" in task_config:
        if task_config["active"].lower() != "true":
            rospy.loginfo("Ignoring task definition, as it was set to inactive")
            return []
    else:
        rospy.loginfo("Task does not have the active key defined. Assuming that the task should be added to the routine.")

    # make sure that this task definition is valid - it must define at
    # minimum waypoints, action and duration fields.
    if "waypoints" not in task_config or "action" not in task_config or "duration" not in task_config:
        rospy.logwarn("Task definition was invalid. A definition must define the fields: waypoints, action,  duration")
        return []

    waypoints, end_waypoints = get_start_end_waypoints(task_config, global_config)

    if waypoints is None and end_waypoints is None:
        return []

    tasks = []
    # create a single task at each waypoint
    for wp_idx, waypoint in enumerate(waypoints):
        task = Task(action=task_config["action"], start_node_id=waypoint, end_node_id=end_waypoints[wp_idx] if end_waypoints else None)
        task.max_duration = rospy.Duration(int(task_config["duration"]))
        # This is required, but is overwritten (is it?) when the task is added as a daily task
        task.start_after = rospy.get_rostime()
        task.end_before = task.start_after + rospy.Duration(5*60) + (task.max_duration * 10)

        task.priority = get_task_priority(task_config)

        # Check if there are additional string, duration or time arguments
        # to process, and add them if there are. This assumes that the
        # dictionaries given in the additional_args list are in the same
        # order as they are in the message that should be sent to start the
        # task. This is something the user has to make sure is correct in
        # the config file.
        if "task_args" in task_config:
            for task_arg_dict in task_config["task_args"]:
                if not arg_dict_is_valid(task_arg_dict):
                    rospy.logwarn("task_args definition should be a dict containing keys: type, value. Actual value: {0}".task_arg_dict)
                    return []

                # if the adding the arg fails, do not add the task
                if not add_task_arg_from_dict(task, waypoint, task_arg_dict, task_config, global_config):
                    return []

        tasks.append(task)

    return tasks

if __name__ == '__main__':
    rospy.init_node("tsc_routine")

    config = rospy.get_param("~routine_config")

    # client = dynamic_reconfigure.client.Client('human_aware_navigation')
    # params = { 'gaze_type' : 0 }
    # config = client.update_configuration(params)

    # Initialise the gaze client to point the eyes towards upper bodies
    # gaze_client = actionlib.SimpleActionClient('gaze_at_pose', GazeAtPoseAction)
    # rospy.loginfo("Waiting for gaze at  pose server...")
    # gaze_client.wait_for_server()
    # rospy.loginfo("... got server")
    # # runtime 0 means indefinite
    # gaze_client.send_goal(GazeAtPoseGoal(runtime_sec=0, topic_name='/upper_body_detector/closest_bounding_box_centre'))

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()

    # initialise operating times
    start = time(config["start_time"]["hours"], config["start_time"]["minutes"], tzinfo=localtz)
    end = time(config["end_time"]["hours"], config["end_time"]["minutes"], tzinfo=localtz)

    # initialise routine
    routine = AutomatedRoutine(daily_start=start, daily_end=end, idle_duration=rospy.Duration(config["idle_duration"]), pre_start_window=timedelta(seconds=config["pre_start_window"]), charging_point=config["charging_points"])

    ##############################################################################
    # Off days
    ##############################################################################
    for day_off in config["days_off"]:
        routine.runner.add_day_off(day_off)

    for date_off in get_off_date_list(config["dates_off"]):
        routine.runner.add_date_off(date_off)

    # Here, we create all the sets of tasks that will be added to the routine.
    for task_name in config["task_definitions"]:
        rospy.loginfo("Creating task set {0}".format(task_name))

        if "routine_times" not in config["task_definitions"][task_name]:
            rospy.logwarn("No key \"routine_times\" specified in config. Will not add {0} tasks to routine.".format(task_name))
            continue

        routine_times = config["task_definitions"][task_name]["routine_times"]
        delta = None if "repeat_delta" not in routine_times else timedelta(**routine_times["repeat_delta"])
        routine.create_task_routine(tasks_from_config(config["task_definitions"][task_name], config),
                                    daily_start = time(routine_times["start_time"]["hours"],
                                                       routine_times["start_time"]["minutes"], tzinfo=localtz),
                                    daily_end = time(routine_times["end_time"]["hours"],
                                                     routine_times["end_time"]["minutes"], tzinfo=localtz),
                                    repeat_delta = delta)

    routine.start_routine()

    client = dynamic_reconfigure.client.Client('automated_routine')

    speed_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
    speed_client.update_configuration({'max_vel_x': 0.45, 'max_trans_vel':0.45})
    rospy.loginfo('Routine running')

    ##############################################################################
    # Battery levels
    ##############################################################################

    # to leave a threshold once entered, battery needs to pass 1v above the threshold
    # levels need to be set 30 minutes prior to anything important happening to ensure the added tasks make it through
    battery_levels = []
    for level_def in config["battery_levels"]:
        battery_levels.append([time(level_def["start_time"]["hours"],
                                    level_def["start_time"]["minutes"], tzinfo=localtz),
                               level_def["hard"], level_def["soft"]])

    run_battery_levels(battery_levels, tz = localtz)
