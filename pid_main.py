#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import pid_controller

#written by Ian, revised by JJ


class PIDMain():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # set the rate of ros loop
        self.rate = rospy.Rate(10) # 10hz
        # Create variable for use
        self.vel_twist = TwistStamped()

        #Creates old time for use with the PIDs
        self.old_time = rospy.get_time()

        # Create object of PID Controller with tunning arguments
        self.pid_methods = pid_controller.PIDController(KP=1, KI=0.0, KD=0.5)  # PID contants P, I, D

        # runs the loop function
        self.loop_roomba_location()  # Runs roomba subscriber

    def loop_roomba_location(self):
        # while the node is still running loop

        # create and subscribe to the message /roomba/location
        self.subscriber = rospy.Subscriber("/Master/control/error", Float64MultiArray, self.callback)

        while not rospy.is_shutdown():
            # publish the accel vector to mavros
            self.pub1.publish(self.vel_twist)  # Vector3Stamped type variable

            # sleep the ros rate
            self.rate.sleep()
            #rospy.spin()

    # runs every time the subcriber above runs
    def callback(self, IncomingData):
        # Method calls that send the XYZ location of the roomba and return xyz acceleration

        IncomingData = IncomingData.data

        dt = rospy.get_time() - self.old_time
        self.oldtime = rospy.get_time()

        if IncomingData[0] == 0:
            x_vel = IncomingData[3]
        else:
            x_vel = self.pid_methods.pid_x(IncomingData[3], dt)

        if IncomingData[1] == 0:
            y_vel = IncomingData[4]
        else:
            y_vel = self.pid_methods.pid_y(IncomingData[4], dt)

        if IncomingData[2] == 0:
            z_vel = IncomingData[5]
        else:
            z_vel = self.pid_methods.pid_z(IncomingData[5], dt)


        # set linear to value
        self.vel_twist.twist.linear.x = x_vel
        self.vel_twist.twist.linear.y = y_vel
        self.vel_twist.twist.linear.z = z_vel * 6

        # Set angular to zero
        self.vel_twist.twist.angular.x = 0
        self.vel_twist.twist.angular.y = 0
        self.vel_twist.twist.angular.z = 0

        # logs the xyz accel data
        rospy.loginfo('--------------------------------')
        rospy.loginfo("incoming data: " + str(IncomingData))
        rospy.loginfo("outgoing data: " + str(x_vel) + ", " + str(y_vel) + ", "+ str(z_vel))


        # publish the accel vector to mavros
        self.pub1.publish(self.vel_twist)  # Vector3Stamped type variable


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('PID_main', anonymous=False)
    try:
        pid = PIDMain()
    except rospy.ROSInterruptException:
        pass

# TO find the length on the ground in meters
    # (height in meters times the distance in pixels)/720
