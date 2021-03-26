#! /usr/bin/ python3

import rospy
from std_msgs import String, Bool, Int32, Float64
from nav_msgs import Odometry
from geometry_msgs import TwistWithCovarianceStamped
from sensor_msgs import Imu


class PythonNode():
    def __init__(self):
        # declare params, subscriber and publisher
        myVar1_ = int(rospy.get_param('~myVar1_', '1'))
        myVar2_ = float(rospy.get_param('~myVar2_', '1.0'))
        timerFreq_ = float(rospy.get_param('~timerFreq_', '20'))

        imuSub_ = rospy.Subscriber('/imu/data', Imu, imuCallback)
        odomSub_ = rospy.Subscriber('/odom', Odometry, odomCallback)

        vehicleCmdPub_ = rospy.Publisher(
            '/vehicle_cmd', TwistWithCovarianceStamped, queue_size=5)

        # declare member variables
        currImu_ = Imu()
        currVel_ = 0.0
        vehicleCmdMsg_ = TwistWithCovarianceStamped()

        while not rospy.is_shutdown():
            self.run()
            rospy.spin()
            rospy.sleep(1.0/self.timerFreq_)

    def run(self):
        # do something cool
        self.vehicleCmdMsg_.twist.twist.linear.x = self.currVel_
        self.vehicleCmdPub_.publish(self.vehicleCmdMsg_)

    def imuCallback(self, msg: Imu) -> None:
        self.currImu_ = msg

    def odomCallback(self, msg: Odometry) -> None:
        self.currVel_ = msg.twist.twist.linear.x


if __name__ == 'main':
    rospy.init_node('python_node')
    try:
        node = PythonNode()
    except rospy.ROSInitException:
        pass
