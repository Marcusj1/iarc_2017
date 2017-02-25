#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *


#Author: JJ Marcus
#Last Edit Date: 29/10/2016


class SimpTakeOff():
    def __init__(self):
        #Creates the publisher for ROS. (name of message, data type(from geometry_msgs.msg), queue size)
        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        #sets the rate of the loop
        self.rate = rospy.Rate(10)

        #creates the altitude variable
        self.altitude = Float64()

        self.goal_altitude = 1

        # Create variable to publish
        self.Velocity_Vector = TwistStamped() # ros data type
        self.main_loop()

    def main_loop(self):
        seq_counter = 0
        rospy.Subscriber("/mavros/altitude",Altitude,self.altitude_callback)
        z_linear = 0
        while not rospy.is_shutdown():
            seq_counter += 1

            # get the altitude
            rospy.Subscriber("/mavros/altitude",Altitude,self.altitude_callback)

            # set the speed
            if self.altitude >= self.goal_altitude - 0.1 and self.altitude <= self.goal_altitude + 0.1 :
                z_linear = 0
            elif self.altitude > self.goal_altitude + 0.1:
                z_linear = - 0.02
            elif self.altitude < self.goal_altitude - 0.1:
                z_linear = 0.02

           # publish the velocity
            self.publish_velocity(0,0,z_linear,0,0,0,seq_counter)

            self.rate.sleep()



    # pubhlshes the velocity
    def publish_velocity(self, x_linear, y_linear, z_linear, x_ang, y_ang, z_ang, seq_counter):

        # set linear to value
        self.Velocity_Vector.twist.linear.x = x_linear
        self.Velocity_Vector.twist.linear.y = y_linear
        self.Velocity_Vector.twist.linear.z = z_linear

        # Set angular to zero
        self.Velocity_Vector.twist.angular.x = x_ang
        self.Velocity_Vector.twist.angular.y = y_ang
        self.Velocity_Vector.twist.angular.z = z_ang
        # Get the time now for the velocity commands
        time = rospy.Time.now()
        self.Velocity_Vector.header.stamp.secs = int(time.to_sec())
        self.Velocity_Vector.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
        self.Velocity_Vector.header.seq = seq_counter

        # logs the xyz accel data
        #rospy.loginfo(self.Velocity_Vector)
        log = [x_linear,y_linear,z_linear,x_ang,y_ang,z_ang, self.altitude]

        rospy.loginfo(log)


        # use the publisher
        self.velocity_publisher.publish(self.Velocity_Vector)

    def altitude_callback(self, data):
        IncomingData = [0,0,0,0,0]
        IncomingData[0] = data.monotonic
        IncomingData[1] = data.amsl
        IncomingData[2] = data.relative
        IncomingData[3] = data.terrain
        IncomingData[4] = data.bottom_clearance

        if IncomingData[2] > 0.35 and IncomingData[2] < 20:
            self.altitude = ( IncomingData[2] )
        else:
            self.altitude = ( IncomingData[1] - IncomingData[3] )


if __name__ == '__main__':
    # initate the node
    rospy.init_node('SimpleMotorTest')
    try:
        test = SimpTakeOff()
    except rospy.ROSInterruptException:
        pass
