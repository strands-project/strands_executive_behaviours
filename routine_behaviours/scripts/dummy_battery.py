#!/usr/bin/env python

import rospy

from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

class DummyBattery(object):
    """
    Publishes a dummy battery message which charges or discharges based on current topoligcal noe
    """
    def __init__(self):
        super(DummyBattery, self).__init__()
        self._charging_points = rospy.get_param('~charging_points', ['ChargingPoint', 'ChargingPoint1', 'ChargingPoint2' ])
        self._current_node = None
        
        # run at 10hz which matches the scitos robot
        self._rate = 10

        # battery percent per second
        self._discharge_rate = float(rospy.get_param('~discharge_rate', 0.03)) / self._rate
        self._recharge_rate = float(rospy.get_param('~recharge_rate', 0.30)) / self._rate

        # battery charge between 1 and 100
        self._current_level = 100

        rospy.Subscriber('/current_node', String, self._update_topological_location)
        self._battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size = 1)



    def _update_topological_location(self, node_name):
        self._current_node = node_name.data

    def run(self):
        rate = rospy.Rate(self._rate)

        msg = BatteryState()

        while not rospy.is_shutdown():
            msg.header.stamp = rospy.get_rostime()
            msg.present = self._current_node in self._charging_points

            if msg.present: 
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                self._current_level = min(100, self._current_level + self._recharge_rate)
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
