#!/usr/bin/env python


import rospy
import random
import datetime
import threading
from region_observation.util import get_soma_info
from activity_simulation.msg import ActivityMsg, ActivitiesMsg
from activity_simulation.srv import ActivitiesSrv, ActivitiesSrvResponse


class ActivityGenerator(object):

    def __init__(self, name, soma_config):
        """
            Generate random activities, at random times, and random places
        """
        rospy.loginfo("Initiating activity generator...")
        self._stop = True
        self._lock = threading.Lock()
        self.available_activities = ["Eat", "Drink", "Other"]
        self._counter_uuid = 1
        self.scheduled_activities = list()
        self._time_ahead = rospy.Duration(60)
        self.regions = get_soma_info(soma_config)[0].keys()
        # self.available_activities = ["Eat", "Drink", "Rest", "Work", "Other"]
        self.act_srv = rospy.Service(
            "%s/available_activities" % name, ActivitiesSrv, self._cb
        )
        rospy.sleep(0.1)
        self.act_pub = rospy.Publisher(
            "%s/activities" % name, ActivitiesMsg, queue_size=10
        )
        rospy.sleep(0.1)

    def _cb(self, msg):
        return ActivitiesSrvResponse(self.available_activities)

    def _update(self):
        activity_locations = dict()
        while (len(self.scheduled_activities) > 0 or not self._stop) and not rospy.is_shutdown():
            activities = ActivitiesMsg()
            current_time = rospy.Time.now()
            for ind, act in enumerate(self.scheduled_activities):
                if str(act[0]) not in activity_locations:
                    activity_locations[str(act[0])] = self.regions[
                        random.randint(0, len(self.regions)-1)
                    ]
                ongoing = False
                if current_time >= act[2] and current_time <= act[2] + act[3]:
                    ongoing = True
                if current_time >= act[2]:
                    activity = ActivityMsg(
                        str(act[0]), act[1], act[2], act[2]+act[3], ongoing,
                        activity_locations[str(act[0])]
                    )
                    activities.activities.append(activity)
                self._lock.acquire_lock()
                if current_time > act[2]+act[3]:
                    del self.scheduled_activities[ind]
                    del activity_locations[str(act[0])]
                self._lock.release_lock()
            self.act_pub.publish(activities)
            rospy.sleep(1)

    def generate(self, start, end):
        now = rospy.Time.now()
        self._stop = False
        self._thread = threading.Thread(target=self._update)
        self._thread.start()
        while (start <= now and now <= end) and not rospy.is_shutdown():
            if len(self.scheduled_activities) < 3:
                self.get_activity_scheduled()
            rospy.sleep(60)
            now = rospy.Time.now()
        self._stop = True
        self._thread.join()

    def get_activity_scheduled(self):
        random.seed(rospy.Time.now().secs)
        if random.random() > 0.5:
            scheduled_activity = self.available_activities[random.randint(
                0, len(self.available_activities)-1
            )]
            scheduled_time = rospy.Time.now() + self._time_ahead
            if scheduled_activity in ["Drink", "Other"]:
                scheduled_duration = rospy.Duration(60*random.randint(1, 5))
            elif scheduled_activity == "Eat":
                scheduled_duration = rospy.Duration(60*random.randint(5, 15))
            else:
                scheduled_duration = rospy.Duration(60*random.randint(30, 120))

            rospy.loginfo(
                "Activity %s is scheduled on %s for %d secs" % (
                    scheduled_activity,
                    datetime.datetime.fromtimestamp(scheduled_time.secs),
                    scheduled_duration.secs
                )
            )
            self._lock.acquire_lock()
            self.scheduled_activities.append(
                (
                    self._counter_uuid, scheduled_activity,
                    scheduled_time, scheduled_duration
                )
            )
            self._lock.release_lock()
            self._counter_uuid += 1
