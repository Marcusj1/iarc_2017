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
            [ 0 ],
            [ 2 ],
            [ 6 ],
            [ 4 ],
            [ 5 ],
            [ 2 ],
            [ 4 ],
            [ 5 ],
            [ 2 ],
            [ 1 ],
            [ 3 ],
        ]
        self.program_index = -1
        ################

        ################ Stance variables
        self.stance = 0
        self.stance_names = [
            'Logical_State',        # 0
            'Land',                 # 1
            'Takeoff',              # 2
            'Stance Disarm',        # 3
            'hover_over_romba',     # 4
            'Land_Over_Roomba',     # 5
            'Left_Right_Slide',     # 6
            'Return_Home'           # 7
        ]

        self.LOGICAL_STANCE    =    0
        self.LAND              =    1
        self.TAKOFF            =    2
        self.STANCE_DISARM     =    3
        self.HOVER_OVER_ROOMBA =    4
        self.LAND_OVER_ROOMBA  =    5
        self.LEFT_RIGHT_SLIDE  =    6
        self.RETURN_HOME       =    7
        ################

        ################ subscriber threads
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)
        rospy.Subscriber("mavros/px4flow/ground_distance",  Range,          self.flownar)
       #rospy.Subscriber("mavros/altitude",                 Altitude,       self.altitude_callback)
        rospy.Subscriber("/mavros/state",                   State,          self.state_callback)
        rospy.Subscriber("/Vision/roomba_location_meters",         PoseArray,      self.roomba_location_callback)
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
        self.master_to_pid_vector.data.append(0)

        self.PID_ON  = 1 # Position control
        self.PID_OFF = 0 # Velocity control (still is pid'd right now)


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
        self.system_position = [0, 0]
        self.old_time = int(time.time())
        ################
        self.mission_time_publisher = rospy.Publisher('master/mission_time', Float32, queue_size=10)
        self.mission_time = Float32()

        ################
        self.rate = rospy.Rate(10)
        time.sleep(1)
        self.preflight()
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
                self.Takeoff(0.55)

            elif self.stance == self.STANCE_DISARM:
                self.Stance_Disarm()

            elif self.stance == self.HOVER_OVER_ROOMBA:
                self.hover_over_roomba(0.55)

            elif self.stance == self.LAND_OVER_ROOMBA:
                self.Land_Over_Roomba(0.55)

            elif self.stance == self.LEFT_RIGHT_SLIDE:
                self.Left_Right_Slide(0.55)

            elif self.stance == self.RETURN_HOME:
                self.Left_Right_Slide(0.55)


            self.master_to_pid_vector.data[0] =   self.x_bool
            self.master_to_pid_vector.data[1] =   self.y_bool
            self.master_to_pid_vector.data[2] =   self.z_bool
            self.master_to_pid_vector.data[3] =   self.x_linear
            self.master_to_pid_vector.data[4] =   self.y_linear
            self.master_to_pid_vector.data[5] = - self.z_linear
            self.master_to_pid_vector.data[6] =   count

            self.rate.sleep()

            self.master_to_pid_publisher.publish(self.master_to_pid_vector)

            self.mission_time.data = count
            self.mission_time_publisher.publish(self.mission_time)

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
        self.countup = 0


        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = 0
    ###################################

    ###################################
    def Land(self):

        if not self.stance_previous == self.LAND:
            self.countdown = 1

        if self.altitude_current < 0.5:
            self.countdown -= 0.1
            rospy.loginfo("low altitude, switching stance in: " + str(self.countdown))
        else:
            self.countdown = 1

        if self.altitude_current < 0.4 and self.countdown < 0:
            self.stance = self.LOGICAL_STANCE

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_ON

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear =  self.maintain_altitude(0.01)
    ###################################

    ###################################
    def Takeoff(self, goal_altitude):

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_ON

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = self.maintain_altitude(goal_altitude)

        if self.altitude_current > goal_altitude-0.05 and self.altitude_current < goal_altitude+0.05:
            self.stance = self.LOGICAL_STANCE

    ###################################

    ###################################
    def Stance_Disarm(self):

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = -9000
        self.disarm()
    ###################################

    ###################################
    def hover_over_roomba(self, goal_altitude):
        if len(self.roomba_list) > 0:
            self.x_bool = self.PID_ON
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_ON
            #rospy.loginfo(str(self.roomba_list))
            self.x_linear = self.roomba_list[0][0]
            self.y_linear = self.roomba_list[0][1]
            #rospy.loginfo(str(self.roomba_list))
            if self.roomba_list[0][2] < 0.1:
                rospy.loginfo("I am over roomba, switching stane in : " + str(self.countdown))
                self.countdown -= 0.1
                if self.countdown < 0:
                   rospy.loginfo("Currently over roomba")
                   self.stance = self.LOGICAL_STANCE

                # This is for the endurence test
                if self.battery_voltage < 13.5:
                     self.stance = self.LOGICAL_STANCE

            else:
                self.countdown = 2
        else:
            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_ON

            self.x_linear = 0
            self.y_linear = 0

            rospy.loginfo("No Roomba In list")

        self.z_linear = self.maintain_altitude(goal_altitude)
        #rospy.loginfo(str(self.x_linear) + " ::: " + str(self.y_linear))
    ###################################

    ###################################
    def Land_Over_Roomba(self,hover_alt):
        if len(self.roomba_list) > 0:
            self.x_bool = self.PID_ON
            self.y_bool = self.PID_ON
            self.z_bool = self.PID_OFF

            self.x_linear = self.roomba_list[0][0]
            self.y_linear = self.roomba_list[0][1]

            if self.roomba_list[0][2] < 0.15:
                rospy.loginfo("I am over roomba, landing")
            if self.altitude_current < 0.4:
                rospy.loginfo("Switching to logical")
                self.stance = self.LOGICAL_STANCE
            self.z_linear   = -0.2
            self.countdown  =  1
        else:
            rospy.loginfo("Roomba Lost ...")
            self.countdown -= 0.1

            if self.countdown < 0:
                 self.stance = self.LOGICAL_STANCE
                 self.program_index = 1
            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_ON

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = self.maintain_altitude(hover_alt)

    ###################################
    def Left_Right_Slide(self, altitude_goal):

            self.countdown  -= 0.1

            if self.countdown < -2:
                self.countdown = 2

            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_ON

            self.y_linear = 0
            self.x_linear = -0.1 * self.countdown / abs(self.countdown)
            self.z_linear = self.maintain_altitude(altitude_goal)

            if len(self.roomba_list) > 0:
                self.stance = self.LOGICAL_STANCE

    ###################################

    ###################################
    def Return_Home(self, altitude_goal):

            self.y_bool = self.PID_ON
            self.x_bool = self.PID_ON
            self.z_bool = self.PID_ON

            self.x_linear = self.system_position[0]
            self.y_linear = self.system_position[1]
            self.z_linear = self.maintain_altitude(altitude_goal)

            if check_within_bounds(self.system_position[0], 0.1, 0) and check_within_bounds(self.system_position[1], 0.1, 0) > 0:
                self.stance = self.LOGICAL_STANCE
    ###################################

    ###################################
    def Object_Avoidence(self, altitude_goal):
         pass


    ##########################################################################################
    ################# various functions ######################################################
    ##########################################################################################

    ###################################
    def preflight(self):
        rospy.loginfo("What is the Flow sensor XY Magnitude?: ")
        rospy.loginfo(str(self.velocity_current_magnitude))
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Is PID running?: ")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Is Circle detect working with camera?: ")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Wiring Check... I2C")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Wiring Check... Battery")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Wiring Check... USB")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Area Check... Is it safe to fly?")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Battery voltage check")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Barrier Erected?: ")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Saftey Glasses on?: ")
        rospy.loginfo(str(raw_input("                                >>>")))
        rospy.loginfo("Are you ready to die? ")
        rospy.loginfo(str(raw_input("                                >>>")))
        return

    ###################################
    def safty_checks(self, count):

        if (((count+1) % 5) == 1 or not (self.stance_previous == self.stance or count < 10)) and self.state_current.guided:
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


        ################# Position tracking

        change_in_time = int(time.time()) - self.old_time
        self.old_time =  int(time.time())
    ###################################

    ###################################
    def maintain_altitude(self, altitude_goal):
        return -(altitude_goal -self.altitude_current)
    ###################################

    ###################################
    def disarm(self):
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

    ###################################
    def check_within_bounds(self, position, bound, goal):
        if position < goal + bound and position > goal - bound:
            return True
        else:
            return False
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
