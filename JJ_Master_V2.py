#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from sensor_msgs.msg import *
import time

#Author: JJ Marcus
#Last Edit Date: 28/11/2016

class Master():

    def __init__(self):


        ################# Program variables
        self.program = [
            [ 0 ],
            [ 6 ],
            [ 2 ],
            [ 1 ]
        ]
        self.program_index = -1
        ################

        ################ Stance variables
        self.stance = 0
        self.stance_names = [
            'Logical_State',        # 0
            'Land',                 # 1
            'Takeoff',              # 2
            'calibrate_fcu',        # 3
            'hover_over_romba',     # 4
            'Land_Over_Roomba',     # 5
            'Left_Right_Slide'      # 6
        ]

        self.LOGICAL_STANCE    =    0
        self.LAND              =    1
        self.TAKOFF            =    2
        self.CALIBRATE_FCU     =    3
        self.HOVER_OVER_ROOMBA =    4
        self.LAND_OVER_ROOMBA  =    5
        self.LEFT_RIGHT_SLIDE  =    6
        ################

        ################ subscriber threads
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)
        rospy.Subscriber("mavros/px4flow/ground_distance",  Range,          self.flownar)
       #rospy.Subscriber("mavros/altitude",                 Altitude,       self.altitude_callback)
        rospy.Subscriber("/mavros/state",                   State,          self.state_callback)
        rospy.Subscriber("/roomba/location_meters",         PoseArray,      self.roomba_location_callback)
        rospy.Subscriber("/mavros/battery",                 BatteryStatus,  self.battery_callback)
        ################

        ################ publisher objects and variables
        self.master_to_pid_publisher = rospy.Publisher("/Master/control/error", Float64MultiArray, queue_size=10)
        self.master_to_pid_vector = Float64MultiArray()
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)

        self.PID_ON  = 1
        self.PID_OFF = 0

        self.x_velocity_offset = 0
        self.y_velocity_offset = 0
        self.z_velocity_offset = 0

        self.state_publisher = rospy.Publisher('mavros/state', State, queue_size = 10)
        self.state_variable = State()
        ################

        ################
        self.roomba_list = []
        self.time_start = time.time
        self.countdown = 5
        ################
        self.rate = rospy.Rate(10)
        time.sleep(1)
        self.main()


    ###################################
    def main(self):
        count = 0
        rospy.loginfo('beginning loop')
        while not rospy.is_shutdown():

            count += 1

            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_OFF

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = 0

            self.stance_previous = self.stance
            self.safty_checks(count)

            if   self.stance == self.LOGICAL_STANCE:
                self.Logical_Stance()

            elif self.stance == self.LAND:
                self.Land()

            elif self.stance == self.TAKOFF:
                self.Takeoff(1.15)

            elif self.stance == self.CALIBRATE_FCU:
                self.calibrate_fcu()

            elif self.stance == self.HOVER_OVER_ROOMBA:
                self.hover_over_roomba(0.75)

            elif self.stance == self.LAND_OVER_ROOMBA:
                self.Land_Over_Roomba()

            elif self.stance == self.LEFT_RIGHT_SLIDE:
                self.Left_Right_Slide(count)

            self.master_to_pid_vector.data[0] = self.x_bool
            self.master_to_pid_vector.data[1] = self.y_bool
            self.master_to_pid_vector.data[2] = self.z_bool
            '''
            if self.x_bool == self.PID_OFF:
                self.x_linear -= self.x_velocity_offset
            if self.y_bool == self.PID_OFF:
                self.y_linear -= self.y_velocity_offset
            if self.z_bool == self.PID_OFF:
                self.z_linear -= self.
            ''' # unnessary if FCU does not have a stupid high offset, which it shouldn't ever.
            self.master_to_pid_vector.data[3] = self.x_linear
            self.master_to_pid_vector.data[4] = self.y_linear
            self.master_to_pid_vector.data[5] =-self.z_linear

            self.rate.sleep()

            self.master_to_pid_publisher.publish(self.master_to_pid_vector)

    ###################################

    ##########################################################################################
    ################# state functions ########################################################
    ##########################################################################################

    ###################################
    def Logical_Stance(self):

        if self.program_index > len(self.program) or self.program_index < 0:
            self.stance = self.LAND
        else:
            self.stance = self.program[self.program_index][0]

        self.program_index += 1

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = 0
    ###################################

    ###################################
    def Land(self):

        if not self.stance_previous == self.LAND:
            self.countdown = 2

        if self.altitude_current < 0.45:
            self.countdown -= 0.1
            rospy.loginfo("WARNING: low altitude, DISARMING in: " + str(self.countdown))
        else:
            self.countdown = 1

        if self.altitude_current < 0.4 and self.countdown < 0:
            self.disarm()

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_ON

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear =  self.maintain_altitude(0.2)
    ###################################

    ###################################
    def Takeoff(self, goal_altitude):

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_ON

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = self.maintain_altitude(goal_altitude)

        if self.altitude_current > goal_altitude-0.1 and self.altitude_current < goal_altitude+0.1:
            self.stance = self.LOGICAL_STANCE

    ###################################

    ################################### THIS SHOULDNT BE NESSARY ANYMORE
    def calibrate_fcu(self):

        self.calibration_count = 0
        self.calibration_nessary = 10

        self.flow_x += self.velocity_current.linear.x
        self.flow_y += self.velocity_current.linear.y
        self.flow_z += self.velocity_current.linear.z


        rospy.loginfo('current flow stat: (x,y,z)')
        rospy.loginfo(self.velocity_current.linear.x)
        rospy.loginfo(self.velocity_current.linear.y)
        rospy.loginfo(self.velocity_current.linear.z)


        self.calibration_count += 1

        if self.calibration_count >= self.calibration_nessary:
            self.x_velocity_offset = self.flow_x / 10
            self.y_velocity_offset = self.flow_y / 10
            self.z_velocity_offset = self.flow_z / 10
            rospy.loginfo('offsets:')
            rospy.loginfo (self.x_velocity_offset)
            rospy.loginfo (self.y_velocity_offset)
            rospy.loginfo (self.z_velocity_offset)
            time.sleep(10)
            self.stance = self.LOGICAL_STANCE

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = 0
    ###################################

    ###################################
    def hover_over_roomba(self, goal_altitude):
        if len(self.roomba_list) > 0:
            self.x_bool = self.PID_ON
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_OFF

            self.x_linear = -self.roomba_list[0][0]
            self.y_linear = -self.roomba_list[0][1]
            self.z_linear = 0

            if self.roomba_list[0][2] < 0.1:
                rospy.loginfo("I am over roomba, switching stane in : " + str(self.countdown))
                self.countdown -= 0.1
                if self.countdown < 0:
                    self.stance = self.LOGICAL_STANCE
            else:
                self.countdown = 0.5
        else:
            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_OFF

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = 0

        self.y_linear = self.maintain_altitude(goal_altitude)
    ###################################

    ###################################
    def Land_Over_Roomba(self):
        if len(self.roomba_list) > 0:
            self.x_bool = self.PID_ON
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_OFF

            self.x_linear = -self.roomba_list[0][0]
            self.y_linear = -self.roomba_list[0][1]

            if self.roomba_list[0][3] < 0.1:
                rospy.loginfo("I am over roomba, landing")
            if self.altitude_current < 0.4:
                rospy.loginfo("DISARM")
                self.disarm()

            self.z_linear = self.maintain_altitude(0.4)

            self.countdown = 3
        else:
            rospy.loginfo("Roomba Lost ...")
            self.countdown -= 0.1

            if self.countdown < 0:
                self.program_index = 0

            self.x_bool = self.PID_OFF
            self.z_bool = self.PID_OFF
            self.y_bool = self.PID_OFF

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = 0

    ###################################
    def Left_Right_Slide(self, count):

            self.countdown  -= 0.1

            if self.countdown < -1.5:
                self.countdown = 1.5

            self.y_bool = self.PID_OFF
            self.x_bool = self.PID_OFF
            self.z_bool = self.PID_ON

            self.y_linear = -self.velocity_current.twist.linear.y * 1.4
            self.x_linear = -self.velocity_current.twist.linear.x * 1.4
            self.z_linear = self.maintain_altitude(0.75)

            if len(self.roomba_list) > 0:
                self.stance = self.LOGICAL_STANCE

    ###################################


    ##########################################################################################
    ################# various functions ######################################################
    ##########################################################################################

    ###################################
    def safty_checks(self, count):

        if (((count+1) % 5) == 1 or not (self.stance_previous == self.stance) or count < 10) and self.state_current.guided:
            rospy.loginfo("--------------------------------------")
            rospy.loginfo("count:                " +    str(count))
            rospy.loginfo("Current stance:       " +    str(self.stance_names[self.stance]) + ": " + str(self.stance) + ", program: " + str(self.program_index))
            rospy.loginfo("Current altitude:     " +    str(self.altitude_current))
            rospy.loginfo("Current ground speed: " +    str(self.velocity_current_magnitude))
            rospy.loginfo("Current voltage:      " +    str(self.battery_voltage))


            if self.battery_voltage < 14:
                rospy.logwarn("WARNING BATTERY VOLTAGE LOW")

            if self.altitude_current > 3:
                rospy.logwarn("WARNING ALTITUDE TOO HIGH")


        if self.battery_voltage < 12:
            rospy.logwarn("WARNING BATTERY VOLTAGE: LANDING")
            self.stance = self.LAND

        if self.battery_voltage < 10:
            rospy.logfatal("BATTERY VOLTAGE TOO LOW: DISARMING")
            self.disarm()

        if self.velocity_current_magnitude > 100:
            rospy.logfatal("TOO HIGH VELOCITY DETECTED: DISARMING")
            self.disarm()

        if self.altitude_current > 4 :
            rospy.logfatal("ALTITUDE TOO HIGH: DISARMING")
            self.disarm()

        if not self.state_current.guided and (not self.stance_previous == self.stance or (count+1)%5) == 1 or count == 1:
            rospy.logwarn("Manual mode, returning to logical. Program will not run. Count: " + str(count))

        if not self.state_current.guided:
            self.stance = self.LOGICAL_STANCE
            self.program_index = 0
    ###################################

    ###################################
    def maintain_altitude(self, altitude_goal):
        return -(altitude_goal -self.altitude_current)
    ###################################

    ###################################
    def disarm(self):
        while True:
            rospy.logfatal("DISARM-DISARM-DISARM")
            self.state_variable.armed = False
            self.state_variable.guided = False
            self.state_publisher.publish(self.state_variable)
            quit()
    ###################################

    ##########################################################################################
    ################# Subscriber Call Backs ##################################################
    ##########################################################################################

    ###################################
    def altitude_callback(self, IncomingData):

        if IncomingData.relative > 0.3 and IncomingData.relative < 20:
            self.altitude_current = ( IncomingData.relative )
        else:
            self.altitude_current = ( IncomingData.amsl - IncomingData.terrain )
    ###################################

    ###################################
    def flownar(self, IncomingData):

        if not IncomingData.range == 0:
            self.altitude_current = IncomingData.range
    ###################################

    ###################################
    def local_position_velocity_callback(self,velocity_current):
        self.velocity_current = velocity_current
        self.velocity_current_magnitude = (velocity_current.twist.linear.x**2 + velocity_current.twist.linear.y**2)**0.5
    ###################################

    ###################################
    def state_callback(self, state):
        self.state_current = state
    ###################################

    ###################################
    def roomba_location_callback(self, roomba_location_pose_arrary):
        previous_roomba_list = self.roomba_list
        self.roomba_list = []
        #the point message is dist in meteres
        #the orientation message is dist in pixles
        for pose in roomba_location_pose_arrary.poses:
            distance = (pose.position.x**2 + pose.position.y**2)**0.5
            self.roomba_list.append([pose.position.y, pose.position.x, distance])
            # See the memo in the guides folder for why y is first and x is second, and this is switched in the rest of the program
    ###################################

    ###################################
    def battery_callback(self, incoming_data):
        self.battery_voltage = incoming_data.voltage
    ###################################

##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

if __name__ == '__main__':
    # initate the node
    rospy.init_node('Master')
    try:
        test = Master()
    except rospy.ROSInterruptException:
        pass


''' Useful bits of code that'll never be used: (maybe)'


            rospy.loginfo("Foward PID: " + str(self.x_bool))
            rospy.loginfo("Foward    : " + str(self.x_linear))
            rospy.loginfo("Right PID : " + str(self.y_bool))
            rospy.loginfo("Right     : " + str(self.y_linear))
            rospy.loginfo("UP PID    : " + str(self.x_bool))
            rospy.loginfo("UP        : " + str(-self.z_linear))

'''
