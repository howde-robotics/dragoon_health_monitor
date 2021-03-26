#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from dragoon_messages.msg import watchHeartbeat


class HealthMonitor():
    def __init__(self):
        # declare params, subscriber and publisher
        self.myVar1_ = int(rospy.get_param('~myVar1_', '1'))
        self.myVar2_ = float(rospy.get_param('~myVar2_', '1.0'))
        self.timerFreq_ = float(rospy.get_param('~timerFreq_', '20'))

        self.imuSub_ = rospy.Subscriber('/imu/data', Imu, self.imuCallback)
        self.odomSub_ = rospy.Subscriber('/odom', Odometry, self.odomCallback)

        self.vehicleCmdPub_ = rospy.Publisher(
            '/vehicle_cmd', TwistWithCovarianceStamped, queue_size=5)

        # declare member variables
        self.currImu_ = Imu()
        self.currVel_ = 0.0
        self.vehicleCmdMsg_ = TwistWithCovarianceStamped()

        while not rospy.is_shutdown():
            self.run()
            rospy.spin()
            rospy.sleep(1.0/self.timerFreq_)

    def run(self):
        # do something cool
        self.vehicleCmdMsg_.twist.twist.linear.x = self.currVel_
        self.vehicleCmdPub_.publish(self.vehicleCmdMsg_)

    def imuCallback(self, msg):
        self.currImu_ = msg

    def odomCallback(self, msg):
        self.currVel_ = msg.twist.twist.linear.x


if __name__ == 'main':
    rospy.init_node('health_monitor')
    try:
        node = HealthMonitor()
    except rospy.ROSInitException:
        pass
