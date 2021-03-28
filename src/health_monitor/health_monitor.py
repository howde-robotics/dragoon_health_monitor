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
    def __init__(self, node_package, node_type, node_name):
        self.node = rl.core.Node(package=node_package, node_type=node_type, name=node_name)
        self.launcher = rl.scriptapi.ROSLaunch()

    # launch the node
    def launch(self):
        self.launcher.start()
        self.launcher.launch(self.node)


class HealthMonitor():

    heartbeats = ['lidar', 'seek', 'realsense', 'peripheral', 'localize', 'detection']
    totalBeats = len(heartbeats)

    def __init__(self):

        # status array 
        self.healthLastTime = np.zeros(self.totalBeats, dtype=int)
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

        # run the node
        self.rosInterface()

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
        checkError = self.healthTimeDiff > errorThres 
        # see if need to restart the node
        checkRestart = self.healthTimeDiff > restartThres
        # rospy.loginfo(checkRestart)

        # for i in range(len(checkError)):
            # checks if node has stopped publishing heartbeat
            # if checkError[i]:
                # rospy.loginfo("[WATCHDOG] Ding dong the '%s' system is dead" % self.heartbeats[i])


            # restart it
            #TODO: add some mechanism to clean up a node that stopped for whatever reason
            # if checkRestart[i]:
            #     system = self.heartbeats[i]
            #     rospy.loginfo(self.restartNodesFlag)
            #     if system in self.healthDict and self.restartNodesFlag is True:
            #         rospy.loginfo("[WATCHDOG] Relaunching '%s' system" % system)
            #         self.healthDict[system].launch()

        self.outputMsg.status_time = rospy.Time.now()
        self.outputMsg.status_array = checkError
        self.healthPub_.publish(self.outputMsg)


    def rosInterface(self):
        # health monitor rate
        self.timerFreq_ = float(rospy.get_param('~monitor_rate', '20'))
        # these are the sensor hearbeat topics 
        self.lidarTopic = str(rospy.get_param('lidar_beat', '/lidar_beat'))
        self.seekTopic = str(rospy.get_param('seek_beat', '/seek_beat'))
        self.realSenseTopic = str(rospy.get_param('realsense_beat', '/realsense_beat'))

        # these are the real time status updates
        self.peripheralTopic = str(rospy.get_param('peripheral_beat', '/peripheral_beat'))
        self.humanLocalizeTopic = str(rospy.get_param('human_beat', '/human_beat'))
        self.humanDetectionTopic = str(rospy.get_param('detection_beat', '/detection_beat'))

        # the heartbeats to listen to subscribers
        self.lidarSub_ = rospy.Subscriber(self.lidarTopic, watchHeartbeat, self.lidarBeatCallback)
        self.seekSub_ = rospy.Subscriber(self.seekTopic, watchHeartbeat, self.seekCallback)
        self.realSenseSub_ = rospy.Subscriber(self.realSenseTopic, watchHeartbeat, self.realSenseCallback)
        self.peripheralSub_ = rospy.Subscriber(self.peripheralTopic, watchHeartbeat, self.peripheralCallback)
        self.humanLocalizeSub_ = rospy.Subscriber(self.humanLocalizeTopic, watchHeartbeat, self.humanLocalizeCallback)
        self.humanDetectionSub_ = rospy.Subscriber(self.humanDetectionTopic, watchHeartbeat, self.humanDetectionCallback)

        # status publisher
        self.outputMsg = watchStatus()
        self.healthPub_ = rospy.Publisher("/health_status", watchStatus, queue_size=10)


    def callbackHandle(self, msg):
        system = msg.system
        rospy.loginfo(system)
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
