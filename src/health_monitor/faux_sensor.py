import rospy
from dragoon_messages.msg import watchHeartbeat

class fauxSensor(object):
    '''
    A fake sensor object used to test out the health monitor node and restart cabailities
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

        # use the faux sensor name as node name
        rospy.init_node(self.nodeName_)

    def pub(self):
        # msg type contains information to restart node
        self.heartbeatMsg_.node_pkg = "health_monitor"
        # node file
        self.heartbeatMsg_.node_type = "faux_sensor.py"
        # node name
        self.heartbeatMsg_.node_name = self.nodeName_
        if self.heartbeatsPublished_ < self.heartbeatDuration_:
            self.sensorPublisher_.publish()
    
    def run(self):
        while not rospy.is_shutdown():
            self.pub
            rospy.spin()
            rospy.sleep(1.0/self.timerFreq_)

# main run loop
if __name__ == "__main__":
    node = fauxSensor()
    try:
        node.run()    
    except rospy.ROSInitException:
        pass