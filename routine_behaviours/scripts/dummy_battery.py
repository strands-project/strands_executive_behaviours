#!/usr/bin/env python

import rospy

from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Pose
from strands_navigation_msgs.msg import TopologicalMap

class DummyBattery(object):
    """
    Publishes a dummy battery message which charges or discharges based on current topoligcal noe
    """
    def __init__(self):
        super(DummyBattery, self).__init__()
        self._charging_points = rospy.get_param('~charging_points', ['ChargingPoint', 'ChargingPoint1', 'ChargingPoint2' ])
        self._charging_poses = []
        self._pose_tolerance = 0.4
        self._at_charging_point = False
        
        # run at 10hz which matches the scitos robot
        self._rate = 10

        # battery percent per second
        self._discharge_rate = float(rospy.get_param('~discharge_rate', 0.0003)) / self._rate
        self._recharge_rate = float(rospy.get_param('~recharge_rate', 0.0030)) / self._rate

        # battery charge between 0 and 1.0
        self._current_level = 1.0

        self._get_charging_points_poses()
        self._pose_sub = rospy.Subscriber('robot_pose', Pose, self._pose_cb)
        self._battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size = 1)

    def _get_charging_points_poses(self):
        topo_map = rospy.wait_for_message("topological_map", TopologicalMap).nodes
        for entry in topo_map:
            if entry.name in self._charging_points:
                self._charging_poses.append(entry.pose)
       

    def _pose_cb(self, msg):
        for pose in self._charging_poses:
            if abs(msg.position.x - pose.position.x) < self._pose_tolerance and abs(msg.position.y - pose.position.y) < self._pose_tolerance:
                self._at_charging_point = True
                return
        self._at_charging_point = False
            

    def run(self):
        rate = rospy.Rate(self._rate)

        msg = BatteryState()

        while not rospy.is_shutdown():
            msg.header.stamp = rospy.get_rostime()
            msg.present = self._at_charging_point

            if msg.present: 
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                self._current_level = min(1, self._current_level + self._recharge_rate)
            else:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                self._current_level = max(0, self._current_level - self._discharge_rate)

            msg.percentage = float(self._current_level)
            self._battery_pub.publish(msg)            
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("dummy_battery")
    db = DummyBattery()
    db.run()
