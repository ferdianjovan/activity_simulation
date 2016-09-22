#!/usr/bin/env python

import time
import rospy
import datetime
import argparse
from dateutil import parser
from activity_simulation.generator import ActivityGenerator


class ActivitySimulation(object):

    def __init__(self, config):
        self.schedule = [
            ["09:00-12:00", "13:00-17:00"],
            ["09:00-17:00", "13:00-17:00"],
            ["09:00-17:00", "13:00-17:00"],
            ["09:00-17:00", "13:00-17:00"],
            ["09:00-17:00", "13:00-16:00", "18:00-21:00"],
            ["09:00-17:00", "13:00-16:00", "18:00-21:00"],
            ["09:00-17:00", "13:00-16:00", "18:00-21:00"],
        ]
        self.generator = ActivityGenerator(rospy.get_name(), config)

    def simulate(self):
        while not rospy.is_shutdown():
            now = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
            schedule = self.parse_schedule(now.weekday())
            for start_time, end_time in schedule:
                if start_time <= now and now <= end_time:
                    rospy.loginfo(
                        "Simulating activity from %s to %s..." % (start_time, end_time)
                    )
                    st = rospy.Time(time.mktime(start_time.timetuple()))
                    et = rospy.Time(time.mktime(end_time.timetuple()))
                    self.generator.generate(st, et)
                    rospy.loginfo(
                        "Simulation from %s to %s has finished..." % (start_time, end_time)
                    )
            rospy.sleep(60)

    def parse_schedule(self, weekday):
        schedule = list()
        for event_time in self.schedule[weekday]:
            st, et = event_time.split("-")
            st = parser.parse(st)
            et = parser.parse(et)
            schedule.append((st, et))
        return schedule


if __name__ == '__main__':
    rospy.init_node("activity_simulation")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("soma_config", help="Soma configuration")
    args = parser_arg.parse_args()

    system = ActivitySimulation(args.soma_config)
    system.simulate()
