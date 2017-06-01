#!/usr/bin/env python
import rospy
from PIDClass import PID
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import time


# Author: JJ Marcus

class Master:
    def __init__(self):
        print('init')
        ############# Create PID's for each linear dimension of the quadcopter
        # KP, KI, KD, maximum_magnitude, maximum_integral
        self.X_POSITION_PID = PID( 1.2, 1.8,  0.0,   1,  5)
        self.Y_POSITION_PID = PID( 1.2, 1.8,  0.0,   1,  5)
        self.Z_POSITION_PID = PID(  1, 0.1, 0.03,   2.0, 1)
        #2.45 is min on low battery

        self.X_VELOCITY_PID = PID( 1.0, 1, 0.5, 1.2, 03)
        self.Y_VELOCITY_PID = PID( 1.0, 1, 0.5, 1.2, 03)
        self.Z_VELOCITY_PID = PID( .5, 0.01, 0.1, 1.0, 10)
        # KP, KI, KD, maximum_magnitude, maximum_integral

        rospy.Subscriber("/Master/control/error", Float64MultiArray, self.pid_subscriber_callback)
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback)
        self.imu_time = rospy.get_time()
        self.imu_vel = [0,0]
        self.imu_accel = [0,0]
        self.x_imu_offset= 0
        self.y_imu_offset= 0
        self.imu_trust_percentage = 0
        self.imu_data_list = [[],[]]

        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        #guessing at position, this is not at all working
        self.position_publisher= rospy.Publisher('PID_Node/position_guess', Float64MultiArray, queue_size=10)
        self.position_guess = Float64MultiArray()
        self.position_guess.data.append(0)
        self.position_guess.data.append(0)
        self.position_guess.data.append(0)
        self.position_guess.data.append(0)

        self.offset_vector = [ 0 , 0 , 0 ]
        self.velocity_vector = TwistStamped()

        self.count = 0
        self.Recieved_Data_From_Main = False
        self.rate = rospy.Rate(10)

        time.sleep(1)
        ################################################
        rospy.loginfo("calibrating Flow offset...")

        self.x_velocity_offset = 0
        self.y_velocity_offset = 0
        self.z_velocity_offset = -0
        sample_size = 100
        sample_count = 1
        while sample_count < sample_size:
            self.x_velocity_offset += self.velocity_current.twist.linear.x
            self.y_velocity_offset += self.velocity_current.twist.linear.y
            sample_count += 1
            self.rate.sleep()
        self.x_velocity_offset = self.x_velocity_offset/sample_size
        self.y_velocity_offset = self.y_velocity_offset/sample_size

        rospy.loginfo("calibration complete")
        rospy.loginfo("x_velocity_offset : " + str(self.x_velocity_offset))
        rospy.loginfo("y_velocity_offset : " + str(self.y_velocity_offset))
        rospy.loginfo("z_velocity_offset : " + str(self.z_velocity_offset))


        ################################################
        rospy.loginfo("calibrating imu offset...")
        self.x_imu_offset= 0
        self.y_imu_offset= 0
        sample_count = 1
        while sample_count < sample_size:
            self.x_imu_offset += self.imu_accel[0]
            self.y_imu_offset += self.imu_accel[1]
            sample_count += 1
            self.rate.sleep()
        self.x_imu_offset = self.x_imu_offset/sample_size
        self.x_imu_offset = self.x_imu_offset/sample_size


        rospy.loginfo("calibration complete")
        rospy.loginfo("x_imu_offset : " + str(self.x_imu_offset))
        rospy.loginfo("y_imu_offset: " + str(self.y_imu_offset))
        self.x_velocity_previous = 0
        self.y_velocity_previous = 0
        self.imu_trust_percentage
        ################################################

        self.main()

    def main(self):
        self.old_time =rospy.get_time()
        while self.Recieved_Data_From_Main == False:
            self.rate.sleep()


        while not rospy.is_shutdown():
            dt = rospy.get_time() - self.old_time
            self.oldtime = rospy.get_time()
            rospy.logwarn(   "Mission time  :" + str(self.IncomingData[6]) + "----------------")

            #self.velocity_current.twist.linear.x = 0
            #self.velocity_current.twist.linear.y = 0
            self.imu_trust_percentage = 0
            ############### Common filter this bitch #############
            flow_trust =  1-self.imu_trust_percentage
            x_velocity = flow_trust*(self.velocity_current.twist.linear.x) + self.imu_trust_percentage *(self.x_velocity_previous + self.imu_accel[0]*dt)
            y_velocity = flow_trust*(self.velocity_current.twist.linear.y) + self.imu_trust_percentage *(self.y_velocity_previous + self.imu_accel[1]*dt)

            rospy.loginfo("Common Filter X: " + str(x_velocity))
            rospy.loginfo("Common Filter Y: " + str(y_velocity))
            rospy.loginfo("imu X: " + str(self.x_velocity_previous + self.imu_accel[0]*dt))
            rospy.loginfo("imu Y: " + str(self.y_velocity_previous + self.imu_accel[1]*dt))
            #####################################################

            ######################################
            # IMU setup up stuff
            if self.imu_trust_percentage > 0:
                self.imu_trust_percentage -= 0.02

            self.x_velocity_previous = x_velocity
            self.y_velocity_previous = y_velocity


            #######################################
            if self.IncomingData[0]  == 0:
                x_vel = self.X_VELOCITY_PID.run(self.IncomingData[3], self.velocity_current.twist.linear.x)+self.x_velocity_offset
                rospy.loginfo("X Velocity PID:   " + str(x_vel))
            elif self.IncomingData[0] == 1:
                x_vel = self.X_POSITION_PID.run(self.IncomingData[3], 0)+self.x_velocity_offset
                rospy.loginfo("X Position PID:   " + str(x_vel))
            else:
                x_vel = self.X_VELOCITY_PID.run(self.IncomingData[3], self.velocity_current.twist.linear.x)+self.x_velocity_offset
                self.X_VELOCITY_PID.integral = 0
                rospy.logwarn("X Velocity PID:   " + str(x_vel))
                self.x_velocity_offset = self.velocity_current.twist.linear.x


            if self.IncomingData[1] == 0:
                y_vel = self.Y_VELOCITY_PID.run(self.IncomingData[4], self.velocity_current.twist.linear.y)+self.y_velocity_offset
                rospy.loginfo("Y Velocity PID:   " + str(y_vel))
            elif self.IncomingData[1] == 1:
                y_vel = self.Y_POSITION_PID.run(self.IncomingData[4], 0)+self.y_velocity_offset
                rospy.loginfo("Y Position PID:   " + str(y_vel))
            else:
                y_vel = self.Y_VELOCITY_PID.run(self.IncomingData[4], self.velocity_current.twist.linear.y)+self.y_velocity_offset
                self.Y_VELOCITY_PID.integral = 0
                rospy.logwarn("Y Velocity PID:   " + str(y_vel))
                self.y_velocity_offset = self.velocity_current.twist.linear.y


            if self.IncomingData[2] == 0:
                z_vel = self.Z_VELOCITY_PID.run(self.IncomingData[5], self.velocity_current.twist.linear.z)+self.z_velocity_offset
                rospy.loginfo("Z Velocity PID:   " + str(z_vel))
            else:
                z_vel = self.Z_POSITION_PID.run(self.IncomingData[5], 0)+self.z_velocity_offset
                rospy.loginfo("Z Position PID:   " + str(z_vel))


            rospy.loginfo("X integral: " + str(self.X_VELOCITY_PID.integral))
            rospy.loginfo("Y integral: " + str(self.Y_VELOCITY_PID.integral))
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

            #print('Processing ms: ' + str(int((-self.old_time + rospy.get_time()))/100000000))

            self.velocity_publisher.publish(self.velocity_vector)

##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

    ##################################
    def pid_subscriber_callback(self, IncomingData):
        self.Recieved_Data_From_Main = True
        Data = list(IncomingData.data)
        Data[3], Data[4] = self.main_to_pix_refrence_frame(Data[3], Data[4])
        self.IncomingData = Data
    ##################################

    ###################################
    def local_position_velocity_callback(self,velocity_current):
         #velocity_current.twist.linear.x, velocity_current.twist.linear.y = self.main_to_pix_refrence_frame(velocity_current.twist.linear.x, velocity_current.twist.linear.y)
	self.velocity_current = velocity_current
    ###################################

    ###################################
    def imu_callback(self,imu_data):

         x_accel, y_accel = self.main_to_pix_refrence_frame(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y)
         x_accel -= self.x_imu_offset
         y_accel -= self.y_imu_offset
         self.imu_data_list[0].append(x_accel)
         self.imu_data_list[1].append(y_accel)

         iterations = 10

         if len(self.imu_data_list[0]) >= iterations:
             self.imu_accel = [sum(self.imu_data_list[0])/iterations, sum(self.imu_data_list[1])/iterations]
             self.imu_data_list = [[],[]]

    ###################################

    ###################################
    def main_to_pix_refrence_frame(self,velocity_in_x, velocity_in_y):
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
