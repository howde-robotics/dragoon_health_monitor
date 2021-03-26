#! /usr/bin/env python

import rospy
from dragoon_messages.msg import watchHeartbeat

class FauxSensor(object):
    '''
    A fake sensor object used to test out the health monitor node and restart
    '''

    def __init__(self):
        # this is the name of the sensor to imitate, that will go in the 'node_name' field
        self.sensorImitation_ = str(rospy.get_param("faux_sensor_imitation", "lidar"))
        # length of heartbeat duration in heartbeats published
        self.heartbeatDuration_ = int(rospy.get_param("faux_sensor_duration", "100"))
        self.timerFreq_ = float(rospy.get_param('~timerFreq_', '10'))
        self.nodeName_ = "faux_" + self.sensorImitation_ + "_publisher"

        self.heartbeatsPublished_ = 0
        self.heartbeatMsg_ = watchHeartbeat()
        self.sensorPublisher_ = rospy.Publisher(
            "/test_heartbeat",
            watchHeartbeat,
            queue_size=5,
        )

        while not rospy.is_shutdown():
            self.run()
            rospy.sleep(1.0/self.timerFreq_)

    def run(self):
        # msg type contains information to restart node
        self.heartbeatMsg_.node_pkg = "health_monitor"
        # node file
        self.heartbeatMsg_.node_type = "faux_sensor.py"
        # node name
        self.heartbeatMsg_.node_name = self.nodeName_
        # time
        self.heartbeatMsg_.heartbeat_time = rospy.Time.now()

        if self.heartbeatsPublished_ < self.heartbeatDuration_:
            self.sensorPublisher_.publish(self.heartbeatMsg_)
        else:
            rospy.loginfo("Stopped heartbeat...")

        self.heartbeatsPublished_ += 1
    
        


# main run loop
if __name__ == "__main__":
    rospy.init_node("faux_lidar_sensor")
    try:
        node = FauxSensor()
    except rospy.ROSInitException:
        pass