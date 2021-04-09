#! /usr/bin/env python

import rospy
import numpy as np
import roslaunch as rl
from dragoon_messages.msg import watchHeartbeat, watchStatus, Objects
from sensor_msgs.msg import Image, LaserScan, Imu
# from darknet_ros.msg import BoundingBoxes


class HealthMonitor():
    # main health monitor class

    heartbeats = ['lidar', 'seek', 'realsense_rgb', 'realsense_depth', 'transformed_imu', 'localize', 'detection']
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

        # # see if an error has occured
        self.checkError = self.healthTimeDiff < errorThres 

        # publish health status
        self.outputMsg.status_time = rospy.Time.now()
        self.outputMsg.status_array = self.checkError
        self.healthPub_.publish(self.outputMsg)


    def rosInterface(self):
        # global sensor hearbeat topics 
        self.lidarTopic           = str(rospy.get_param('lidar_beat', '/scan'))
        self.seekTopic            = str(rospy.get_param('seek_beat', '/seek_camera/filteredImage'))
        self.realSenseRGBTopic    = str(rospy.get_param('realsense_rgb_beat', '/camera/color/image_raw'))
        self.realSenseDepthTopic  = str(rospy.get_param('realsense_depth_beat', '/camera/depth/image_rect_raw'))
        self.transformedIMUTopic  = str(rospy.get_param('transform_imu_beat', '/imu'))

        # global real time status update topics
        self.humanLocalizeTopic  = str(rospy.get_param('human_beat', '/ObjectPoses'))
        self.humanDetectionTopic = str(rospy.get_param('detection_beat', '/darknet_ros/bounding_boxes'))

        # the heartbeats to listen to subscribers
        self.lidarSub_           = rospy.Subscriber(self.lidarTopic, LaserScan, self.lidarBeatCallback)
        self.seekSub_            = rospy.Subscriber(self.seekTopic, Image, self.seekCallback)
        self.realSenseRGBSub_    = rospy.Subscriber(self.realSenseRGBTopic, Image, self.realSenseRGBCallback)
        self.realSenseDepthSub_  = rospy.Subscriber(self.realSenseDepthTopic, Image, self.realSenseDepthCallback)
        self.transformedIMUSub_  = rospy.Subscriber(self.transformedIMUTopic, Imu, self.imuCallback)
        self.humanLocalizeSub_   = rospy.Subscriber(self.humanLocalizeTopic, Objects, self.humanLocalizeCallback)
        # self.humanDetectionSub_ = rospy.Subscriber(self.humanDetectionTopic, BoundingBoxes, self.humanDetectionCallback)

        # status publisher
        self.statusTopic_ = str(rospy.get_param("health_status", "/health_status"))
        self.outputMsg    = watchStatus()
        self.healthPub_   = rospy.Publisher(self.statusTopic_, watchStatus, queue_size=10)


    def callbackHandle(self, msg, system):
        # udpates the system descriptor's array entry with last message receieved time
        sys_index = self.heartbeats.index(system)
        self.healthLastTime[sys_index] = msg.header.stamp.to_nsec()

        # if want to ignore the time
        # self.checkError[sys_index] = True


    # TODO: see if I can use just one callback...
    def lidarBeatCallback(self, msg):
        self.callbackHandle(msg, 'lidar')

    def seekCallback(self, msg):
        self.callbackHandle(msg, 'seek')

    def realSenseRGBCallback(self, msg):
        self.callbackHandle(msg, 'realsense_rgb')

    def realSenseDepthCallback(self, msg):
        self.callbackHandle(msg, 'realsense_depth')

    def imuCallback(self, msg):
        self.callbackHandle(msg, 'transformed_imu')

    def humanLocalizeCallback(self, msg):
        self.callbackHandle(msg, 'localize')

    def humanDetectionCallback(self, msg):
        self.callbackHandle(msg, 'detection')


if __name__ == '__main__':
    rospy.init_node('health_monitor')
    try:
        node = HealthMonitor()
    except rospy.ROSInitException:
        pass
