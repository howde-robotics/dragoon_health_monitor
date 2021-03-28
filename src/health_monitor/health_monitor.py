#! /usr/bin/env python

import rospy
import numpy as np
import roslaunch as rl
from enum import IntEnum
from std_msgs.msg import String, Bool, Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from dragoon_messages.msg import watchHeartbeat, watchStatus

class LaunchNode():
    # object using roslaunch API to launch nodes 

    def __init__(self, node_package, node_type, node_name):
        self.node = rl.core.Node(package=node_package, node_type=node_type, name=node_name)
        self.launcher = rl.scriptapi.ROSLaunch()

    # launch the node
    def launch(self):
        self.launcher.start()
        self.launcher.launch(self.node)


class HealthMonitor():
    # main health monitor class

    heartbeats = ['lidar', 'seek', 'realsense', 'peripheral', 'localize', 'detection']
    totalBeats = len(heartbeats)

    def __init__(self):

        # status array 
        self.healthLastTime = np.ones(self.totalBeats, dtype=int)
        self.healthPrevTime = np.zeros(self.totalBeats, dtype=int)
        self.healthTimeDiff = np.zeros(self.totalBeats, dtype=int)
        # fault dictionary
        self.healthDict = {}

        # how long before error is raised in seconds
        self.errorThreshold = float(rospy.get_param('~error_thres', '2.5'))
        # how long before node is restarted
        self.restartThreshold = float(rospy.get_param('~restart_thres', '5'))
        # whether or not to restart
        self.restartNodesFlag = bool(rospy.get_param('~restart_flag', 'False'))
        # health monitor rate
        self.timerFreq_ = float(rospy.get_param('~monitor_rate', '20'))

        # run the node
        self.rosInterface()

        # initialize the last recieved health time at the current time
        curTime = rospy.Time.now()
        self.healthLastTime *= curTime.to_nsec()

        while not rospy.is_shutdown():
            self.run()
            rospy.sleep(1.0/self.timerFreq_)

    def run(self):
        # subtract current times from last times
        curTime = rospy.Time.now()
        self.healthTimeDiff = curTime.to_nsec() - self.healthLastTime
        # rospy.loginfo(self.healthTimeDiff)

        # convert thresholds to nanoseconds
        errorThres = int(self.errorThreshold * 1e9)
        restartThres = int(self.restartThreshold * 1e9)

        # see if an error has occured
        self.checkError = self.healthTimeDiff < errorThres 
        # see if need to restart the node
        self.checkRestart = self.healthTimeDiff < restartThres
        # rospy.loginfo(self.checkRestart)

        for i in range(len(self.checkError)):
            # restart it
            # TODO: add some mechanism to clean up a node that stopped for whatever reason
            if self.checkRestart[i]:
                system = self.heartbeats[i]
                if system in self.healthDict and self.restartNodesFlag is True:
                    rospy.loginfo("[WATCHDOG] Relaunching '%s' system" % system)
                    self.healthDict[system].launch()

        # publish health status
        self.outputMsg.status_time = rospy.Time.now()
        self.outputMsg.status_array = self.checkError
        self.healthPub_.publish(self.outputMsg)


    def rosInterface(self):
        # global sensor hearbeat topics 
        self.lidarTopic     = str(rospy.get_param('lidar_beat', '/lidar_beat'))
        self.seekTopic      = str(rospy.get_param('seek_beat', '/seek_beat'))
        self.realSenseTopic = str(rospy.get_param('realsense_beat', '/realsense_beat'))

        # global real time status update topics
        self.peripheralTopic     = str(rospy.get_param('peripheral_beat', '/peripheral_beat'))
        self.humanLocalizeTopic  = str(rospy.get_param('human_beat', '/human_beat'))
        self.humanDetectionTopic = str(rospy.get_param('detection_beat', '/detection_beat'))

        # the heartbeats to listen to subscribers
        self.lidarSub_          = rospy.Subscriber(self.lidarTopic, watchHeartbeat, self.lidarBeatCallback)
        self.seekSub_           = rospy.Subscriber(self.seekTopic, watchHeartbeat, self.seekCallback)
        self.realSenseSub_      = rospy.Subscriber(self.realSenseTopic, watchHeartbeat, self.realSenseCallback)
        self.peripheralSub_     = rospy.Subscriber(self.peripheralTopic, watchHeartbeat, self.peripheralCallback)
        self.humanLocalizeSub_  = rospy.Subscriber(self.humanLocalizeTopic, watchHeartbeat, self.humanLocalizeCallback)
        self.humanDetectionSub_ = rospy.Subscriber(self.humanDetectionTopic, watchHeartbeat, self.humanDetectionCallback)

        # status publisher
        self.statusTopic_ = str(rospy.get_param("health_status", "/health_status"))
        self.outputMsg    = watchStatus()
        self.healthPub_   = rospy.Publisher(self.statusTopic_, watchStatus, queue_size=10)


    def callbackHandle(self, msg):
        system = msg.system
        # udpates the system descriptor's array entry with last message receieved time
        sys_index = self.heartbeats.index(system)
        self.healthLastTime[sys_index] = msg.heartbeat_time.to_nsec()

        # checks if the dictionary has been populated with a launchnode object for the system
        if system not in self.healthDict:
            self.healthDict[system] = LaunchNode(msg.node_pkg, msg.node_type, msg.node_name)

    # TODO: see if I can use just one callback...
    def lidarBeatCallback(self, msg):
        self.callbackHandle(msg)

    def peripheralCallback(self, msg):
        self.callbackHandle(msg)

    def seekCallback(self, msg):
        self.callbackHandle(msg)

    def realSenseCallback(self, msg):
        self.callbackHandle(msg)

    def humanLocalizeCallback(self, msg):
        self.callbackHandle(msg)

    def humanDetectionCallback(self, msg):
        self.callbackHandle(msg)


if __name__ == '__main__':
    rospy.init_node('health_monitor')
    try:
        node = HealthMonitor()
    except rospy.ROSInitException:
        pass
