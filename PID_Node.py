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
        # KP, KI, KD, maximum_magnitude, maximum_integral
        self.X_POSITION_PID = PID( 1.20, 0.8, 0,   1.,  5)
        self.Y_POSITION_PID = PID( 1.20, 0.8, 0,   1.,  5)
        self.Z_POSITION_PID = PID( 2.00, 0.5, 0,   .6, 10)

        self.X_VELOCITY_PID = PID( 1.0, 0.1, 0.1, 1.0, 1)
        self.Y_VELOCITY_PID = PID( 1.0, 0.1, 0.1, 1.0, 1)
        self.Z_VELOCITY_PID = PID( 2.0, 0.01, 0.1, 1.0, 10)

        rospy.Subscriber("/Master/control/error", Float64MultiArray, self.pid_subscriber_callback)
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)


        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        #guessing at position, this is not yet working
        self.position_publisher= rospy.Publisher('PID_Node/position_guess', Float64MultiArray, queue_size=10)
        self.position_guess = Float64MultiArray()
        self.position_guess.data.append(0)
        self.position_guess.data.append(1)

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
            rospy.logwarn(   "Mission time  :" + str(self.IncomingData[6]) + "----------------")

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

            self.position_guess.data[0] += self.velocity_current.twist.linear.x +0.707106781*(self.Y_VELOCITY_PID.integral- self.X_VELOCITY_PID.integral)
            self.position_guess.data[1] += self.velocity_current.twist.linear.y +0.707106781*(self.Y_VELOCITY_PID.integral+ self.X_VELOCITY_PID.integral)
            # use the publisher
            self.position_publisher.publish(self.position_guess)
            self.velocity_publisher.publish(self.velocity_vector)


##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

    ##################################
    def pid_subscriber_callback(self, IncomingData):
        self.Recieved_Data = True
        Data = list(IncomingData.data)
        Data[3], Data[4] = self.main_to_act_refrence_frame(Data[3], Data[4])
        self.IncomingData = Data
    ##################################

    ###################################
    def local_position_velocity_callback(self,velocity_current):
        #velocity_current.twist.linear.x, velocity_current.twist.linear.y = self.main_to_act_refrence_frame(velocity_current.twist.linear.x, velocity_current.twist.linear.y)
	self.velocity_current = velocity_current
    ###################################

    ###################################
    def main_to_act_refrence_frame(self,velocity_in_x, velocity_in_y):
        velocity_out_x = 0.707106781*(-velocity_in_x + velocity_in_y)
        velocity_out_y = 0.707106781*( velocity_in_x + velocity_in_y)
        return velocity_out_x, velocity_out_y
    ###################################



if __name__ == '__main__':
    # initate the node
    rospy.init_node('PID_Node')
    try:
        test = Master()
    except rospy.ROSInterruptException:
        pass
