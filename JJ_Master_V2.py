#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
import time

#Author: JJ Marcus
#Last Edit Date: 28/11/2016

class Master():

    def __init__(self):


        ################# Program variables
        self.program = [
            [ 2 , 2 ],
            [ 4 , 2 ],
            [ 1 ]
        ]
        self.program_index = 0
        ################

        ################ Stance variables
        self.stance = 0
        self.stance_names = [
            'Logical_State',        # 0
            'Land',                 # 1
            'Takeoff',              # 2
            'calibrate_fcu',        # 3
            'hover_over_romba'      # 4
            'Land_Over_Roomba'      # 5

        ]

        self.LOGICAL_STANCE =       0
        self.LAND =                 1
        self.TAKOFF =               2
        self.CALIBRATE_FCU =        3
        self.HOVER_OVER_ROOMBA =    4
        self.LAND_OVER_ROOMBA =     5
        ################

        ################ subscriber threads
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)
        rospy.Subscriber("mavros/altitude",                 Altitude,       self.altitude_callback)
        rospy.Subscriber("/mavros/state",                   State,          self.state_callback)
        rospy.Subscriber("/roomba/location_meters",         PoseArray,      self.roomba_location_callback)
        ################

        ################ publisher objects and variables
        self.master_to_pid_publisher = rospy.Publisher("/master/control/error", Float64MultiArray, queue_size=10)
        self.master_to_pid_vector = Float64MultiArray()
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)
        self.master_to_pid_vector.data.append(0)

        self.PID_ON =  1
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
        ################

        time.sleep(1)
        self.main()


    ###################################
    def main(self):
        count = 0
        rospy.loginfo('beginning loop')
        while not rospy.is_shutdown():

            count += 1
            #self.safty_checks(count)

            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_OFF

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = 0

            if self.stance == self.LOGICAL_STANCE:
                self.Logical_Stance()

            elif self.stance == self.LAND:
                self.Land()

            elif self.stance == self.TAKOFF:
                self.Takeoff(self.program[self.program_index][1])

            elif self.stance == self.CALIBRATE_FCU:
                self.calibrate_fcu()

            elif self.stance == self.HOVER_OVER_ROOMBA:
                self.hover_over_roomba(self.program[self.program_index][1])

            elif self.stance == self.LAND_OVER_ROOMBA:
                self.Land_Over_Roomba()

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
            ''' # unnessary if FCU does not have a stupid offset, which it shouldn't
            self.master_to_pid_vector.data[3] = self.x_linear
            self.master_to_pid_vector.data[4] = self.y_linear
            self.master_to_pid_vector.data[5] = self.z_linear

            self.stance_previous = self.stance

            self.master_to_pid_publisher.publish(self.master_to_pid_vector)

    ###################################

    ##########################################################################################
    ################# state functions ########################################################
    ##########################################################################################

    ###################################
    def Logical_Stance(self):

        if self.program_index > len(self.program):
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
            countdown = 5

        if self.altitude_current < 0.10 or self.altitude_current < 0:
            countdown -= 0.1
            rospy.loginfo("WARNING: low altitude, DISARMING in: " + str(countdown))
        else:
            countdown = 10

        if ( self.altitude_current < 0.05 or self.altitude_current < 0 ) and countdown < 0:
            self.disarm()

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = -0.1
        self.z_linear = 0
    ###################################

    ###################################
    def Takeoff(self, goal_altitude):

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_ON
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = self.maintain_altitude(goal_altitude)
        self.z_linear = 0
    ###################################

    ################################### THIS SHOULDNT BE NESSARY ANYMORE
    def calibrate_fcu(self):

        self.calibration_count = 0
        self.calibration_nessary = 10

        self.flow_x += self.velocity_current.linear.x
        self.flow_y += self.velocity_current.linear.y
        self.flow_z += self.velocity_current.linear.z


        print('current flow stat: (x,y,z)')
        print(self.velocity_current.linear.x)
        print(self.velocity_current.linear.y)
        print(self.velocity_current.linear.z)


        self.calibration_count += 1

        if self.calibration_count >= self.calibration_nessary:
            self.x_velocity_offset = self.flow_x / 10
            self.y_velocity_offset = self.flow_y / 10
            self.z_velocity_offset = self.flow_z / 10
            print('offsets:')
            print (self.x_velocity_offset)
            print (self.y_velocity_offset)
            print (self.z_velocity_offset)
            time.sleep(10)
            self.stance = 0

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
            self.z_bool = self.PID_ON

            self.x_linear = self.roomba_list[0][0]
            self.z_linear = self.roomba_list[0][1]

            if self.roomba_list[0][3] < 0.1:
                rospy.loginfo("I am over roomba, switching stane in : " + str(self.countdown))
                self.countdown -= 0.1
                if self.countdown < 0:
                    self.stance = 0
            else:
                self.countdown = 5
        else:
            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_OFF

            self.x_linear = 0
            self.z_linear = 0

        self.y_linear = self.maintain_altitude(goal_altitude)
    ###################################

    ###################################
    def Land_Over_Roomba(self):
        if len(self.roomba_list) > 0:
            self.x_bool = self.PID_ON
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_ON

            self.x_linear = self.roomba_list[0][0]
            self.z_linear = self.roomba_list[0][1]

            if self.roomba_list[0][3] < 0.1:
                rospy.loginfo("I am over roomba, landing")
            if self.altitude_current < 0.3:
                rospy.loginfo("DISARM")
                self.disarm()

            self.y_linear = -0.1

            self.countdown = 3
        else:
            print("Roomba Lost ...")
            self.countdown -= 0.1

            if self.countdown < 0:
                self.program_index = 0

            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_OFF

            self.x_linear = 0
            self.z_linear = 0
            self.y_linear = 0

    ###################################

    ##########################################################################################
    ################# various functions ######################################################
    ##########################################################################################

    ###################################
    def safty_check(self, count):
        if not self.state_current.guided:
            self.stance = 0

        if (count % 20) == 1 or not self.stance_previous == self.stance:
            rospy.loginfo("Current stance: " +          str(self.stance_names[self.stance]))
            rospy.loginfo("Current altitude: " +        str(self.altitude_current))
            rospy.loginfo("Current ground speed: " +    str(self.velocity_current_magnitude))

        if self.velocity_current_magnitude > 10:
            self.disarm()

        if self.time_start + 2*60 < time.time:
            self.stance = self.LAND
    ###################################

    ###################################
    def maintain_altitude(self, altitude_goal):
        return self.altitude_current - altitude_goal
    ###################################

    ###################################
    def disarm(self):
        while True:
            rospy.loginfo("DISARM-DISARM-DISARM")
            rospy.logwarn("DISARM-DISARM-DISARM")
            rospy.logfatal("DISARM-DISARM-DISARM")
            self.state_variable.armed = False
            self.state_variable.guided = False
            self.state_publisher.publish(self.state_variable)
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
    def local_position_velocity_callback(self,velocity_current):
        self.velocity_current = velocity_current
        self.velocity_current_magnitude = (velocity_current.linear.x^2 + velocity_current.linear.y^2)^0.5
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
            self.roomba_list.append([pose.position.x, pose.position.y, distance])
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
