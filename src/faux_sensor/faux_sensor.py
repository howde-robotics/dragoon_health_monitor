#! /usr/bin/env python

import rospy
from dragoon_messages.msg import watchHeartbeat

class FauxSensor(object):
    '''
    A fake sensor object used to test out the health monitor node and restart
    '''

    def __init__(self):
        # global sensor hearbeat topics
        self.lidarTopic_     = str(rospy.get_param('lidar_beat', '/lidar_beat'))
        self.seekTopic_      = str(rospy.get_param('seek_beat', '/seek_beat'))
        self.realSenseTopic_ = str(rospy.get_param('realsense_beat', '/realsense_beat'))

        #  global real time status update topics
        self.peripheralTopic_     = str(rospy.get_param('peripheral_beat', '/peripheral_beat'))
        self.humanLocalizeTopic_  = str(rospy.get_param('human_beat', '/human_beat'))
        self.humanDetectionTopic_ = str(rospy.get_param('detection_beat', '/detection_beat'))

        # this is the name of the sensor to imitate, that will go in the 'node_name' field
        self.sensorImitation_ = str(rospy.get_param("~beat_imitate", "lidar"))
        # length of heartbeat duration in heartbeats published
        self.heartbeatDuration_ = int(rospy.get_param("~beat_duration", "100"))
        self.timerFreq_         = float(rospy.get_param('~beat_rate', '10'))
        self.nodeName_          = "faux_" + self.sensorImitation_ + "_publisher"

        beatsToMimic_ = {
            "lidar"     : self.lidarTopic_,
            "seek"      : self.seekTopic_,
            "realsense" : self.realSenseTopic_,
            "peripheral": self.peripheralTopic_,
            "localize"  : self.humanLocalizeTopic_,
            "detection" : self.humanDetectionTopic_
            }

        self.heartbeatsPublished_ = 0
        self.heartbeatMsg_        = watchHeartbeat()
        self.sensorPublisher_     = rospy.Publisher(
            beatsToMimic_[self.sensorImitation_],
            watchHeartbeat,
            queue_size=5,
        )

        while not rospy.is_shutdown():
            self.run()
            rospy.sleep(1.0/self.timerFreq_)

    def run(self):
        # msg type contains information to restart node
        self.heartbeatMsg_.system   = self.sensorImitation_
        self.heartbeatMsg_.node_pkg = "dragoon_health_monitor"
        # node file
        self.heartbeatMsg_.node_type = "faux_sensor.py"
        # node name
        self.heartbeatMsg_.node_name = self.nodeName_
        # time
        self.heartbeatMsg_.heartbeat_time = rospy.Time.now()


        if self.heartbeatsPublished_ < self.heartbeatDuration_:
            self.sensorPublisher_.publish(self.heartbeatMsg_)

        else:
            rospy.loginfo("[FAUX SENSOR] Stopped heartbeat after %d publications." % self.heartbeatsPublished_)
            rospy.signal_shutdown("[FAUX SENSOR] Completed mimicry of %s system." % self.sensorImitation_)

        self.heartbeatsPublished_ += 1
    
        


# main run loop
if __name__ == "__main__":
    rospy.init_node("faux_lidar_sensor")
    try:
        node = FauxSensor()
    except rospy.ROSInitException:
        pass