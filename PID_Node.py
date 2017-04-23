#!/usr/bin/env python
import rospy
from PIDClass import PID
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import time


# Author: JJ Marcus

class Master:
    def __init__(self):
        print('init')
        ############# Create PID's for each linear dimension of the quadcopter
        self.X_POSITION_PID = PID( 1.20, 0.45, 0, 0.55,  5)
        self.Y_POSITION_PID = PID( 1.20, 0.45, 0, 0.55,  5)
        self.Z_POSITION_PID = PID( 1.20, 0.01, 0,   .5, 10)

        self.X_VELOCITY_PID = PID( 1.0, 0.05, 0,   .5, 10)
        self.Y_VELOCITY_PID = PID( 1.0, 0.05, 0,   .5, 10)
        self.Z_VELOCITY_PID = PID( 1.0, 0.01, 0,   .5, 10)

        rospy.Subscriber("/Master/control/error", Float64MultiArray, self.pid_subscriber_callback)
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)

        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        self.offset_vector = [ 0 , 0 , 0 ]
        self.velocity_vector = TwistStamped()

        self.count = 0
        self.Recieved_Data = False
        self.rate = rospy.Rate(10)
        self.main()

    def main(self):
        self.old_time =rospy.get_time()
        while self.Recieved_Data == False:
            pass
        while not rospy.is_shutdown():
            dt = rospy.get_time() - self.old_time
            self.oldtime = rospy.get_time()
            rospy.logwarn(    "Mission time  :" + str(self.IncomingData[6]) + "----------------")

            if self.IncomingData[0]  == 0:
                x_vel = self.X_VELOCITY_PID.run(self.IncomingData[3], self.velocity_current.twist.linear.x)
                rospy.logwarn("X Velocity PID:   " + str(x_vel))
            else:
                x_vel = self.X_POSITION_PID.run(self.IncomingData[3], 0)
                rospy.logwarn("X Position PID:   " + str(x_vel))

            if self.IncomingData[1] == 0:
                y_vel = self.Y_VELOCITY_PID.run(self.IncomingData[4], self.velocity_current.twist.linear.y)
                rospy.logwarn("Y Velocity PID:   " + str(y_vel))
            else:
                y_vel = self.Y_POSITION_PID.run(self.IncomingData[4], 0)
                rospy.logwarn("Y Position PID:   " + str(y_vel))

            if self.IncomingData[2] == 0:
                z_vel = self.Z_VELOCITY_PID.run(self.IncomingData[5], self.velocity_current.twist.linear.z)
                rospy.logwarn("Z Velocity PID:   " + str(z_vel))
            else:
                z_vel = self.Z_POSITION_PID.run(self.IncomingData[5], 0)
                rospy.logwarn("Z Position PID:   " + str(z_vel))



            #rospy.loginfo("incoming data: " + str(self.IncomingData))
            #rospy.loginfo("outgoing data: " + str(x_vel) + ", " + str(y_vel) + ", "+ str(z_vel))
            self.rate.sleep()

            x_ang = 0
            y_ang = 0
            z_ang = 0

            ##########################################################################################
            ################# velocity publisher #####################################################
            ##########################################################################################
            #  set linear to value
            self.velocity_vector.twist.linear.x = x_vel
            self.velocity_vector.twist.linear.y = y_vel
            self.velocity_vector.twist.linear.z = z_vel

            # Set angular to zero
            self.velocity_vector.twist.angular.x = x_ang
            self.velocity_vector.twist.angular.y = y_ang
            self.velocity_vector.twist.angular.z = z_ang

            # Get the time now for the velocity commands
            #clock = rospy.time.now()
            #self.velocity_vector.header.stamp.secs = int(clock.to_sec())
            #self.velocity_vector.header.stamp.nsecs = int((clock.to_sec() - int(clock.to_sec())) * 1000000000)
            #self.velocity_vector.header.seq = self.count

            # use the publisher
            self.velocity_publisher.publish(self.velocity_vector)


##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

    ##################################
    def pid_subscriber_callback(self, IncomingData):
        self.Recieved_Data = True
        self.IncomingData = IncomingData.data
    ##################################

   ###################################
    def local_position_velocity_callback(self,velocity_current):
        self.velocity_current = velocity_current
    ###################################



if __name__ == '__main__':
    # initate the node
    rospy.init_node('PID_Node')
    try:
        test = Master()
    except rospy.ROSInterruptException:
        pass
