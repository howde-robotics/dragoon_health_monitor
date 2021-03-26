#! /usr/bin/env python

import rospy
import numpy as np
from enum import IntEnum
from std_msgs.msg import String, Bool, Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from dragoon_messages.msg import watchHeartbeat


class HealthMonitor():
    
    class HeartBeats():
        # these are the status array indices of each hearbeat
        lidar       = 0
        seek        = 1
        realsense   = 2
        peripheral  = 3
        localize    = 4
        detection   = 5
        
        # total beats
        totalBeats  = 6

    def __init__(self):

        # status array 
        self.healthStatusArray = np.zeros(self.HeartBeats.totalBeats)
        # fault dictionary
        self.faults = {}

        # run the node
        self.rosInterface()

        while not rospy.is_shutdown():
            self.run()
            rospy.sleep(1.0/self.timerFreq_)

    def run(self):
        self.updateStatus()

    def updateStatus(self):
        rospy.loginfo(self.healthStatusArray)
        self.healthStatusArray = np.zeros(self.HeartBeats.totalBeats)

    def rosInterface(self):
        # declare params, subscriber and publisher
        self.timerFreq_ = float(rospy.get_param('monitor_rate', '20'))
        # these are the sensor hearbeat topics 
        self.lidarTopic = str(rospy.get_param('lidar_beat', '/lidar_beat'))
        self.seekTopic = str(rospy.get_param('seek_beat', '/seek_beat'))
        self.realSenseTopic = str(rospy.get_param('realsense_beat', '/realsense_beat'))

        # these are the real time status updates
        self.peripheralTopic = str(rospy.get_param('peripheral_beat', '/peripheral_beat'))
        self.humanLocalizeTopic = str(rospy.get_param('human_beat', '/human_beat'))
        self.humanDetectionTopic = str(rospy.get_param('detection_beat', '/detection_beat'))

        # the heartbeats to listen to
        self.lidarSub_ = rospy.Subscriber(self.lidarTopic, watchHeartbeat, self.lidarBeatCallback)
        self.seekSub_ = rospy.Subscriber(self.seekTopic, watchHeartbeat, self.seekCallback)
        self.realSenseSub_ = rospy.Subscriber(self.realSenseTopic, watchHeartbeat, self.realSenseCallback)
        self.peripheralSub_ = rospy.Subscriber(self.peripheralTopic, watchHeartbeat, self.peripheralCallback)
        self.humanLocalizeTopic = rospy.Subscriber(self.humanLocalizeTopic, watchHeartbeat, self.humanLocalizeCallback)
        self.humanDetectionTopic = rospy.Subscriber(self.humanDetectionTopic, watchHeartbeat, self.humanDetectionCallback)

    def lidarBeatCallback(self, msg):
        self.healthStatusArray[self.HeartBeats.lidar] = 1

    def peripheralCallback(self, msg):
        self.healthStatusArray[self.HeartBeats.peripheral] = 1

    def seekCallback(self, msg):
        self.healthStatusArray[self.HeartBeats.seek] = 1

    def realSenseCallback(self, msg):
        self.healthStatusArray[self.HeartBeats.realsense] = 1

    def humanLocalizeCallback(self, msg):
        self.healthStatusArray[self.HeartBeats.localize] = 1

    def humanDetectionCallback(self, msg):
        self.healthStatusArray[self.HeartBeats.detection] = 1


if __name__ == '__main__':
    rospy.init_node('health_monitor')
    try:
        node = HealthMonitor()
    except rospy.ROSInitException:
        pass
