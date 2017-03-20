#!/usr/bin/env python
import rospy
from PIDClass import PID
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray


# Author: JJ Marcus

class Master():
    def __init__(self):
        print('init')
        ############# Create PID's for each linear dimension of the quadcopter
        self.X_POSITION_PID = PID( 0.3, 0, 0, 1/4)
        self.Y_POSITION_PID = PID( 0.3, 0, 0, 1/4)
        self.Z_POSITION_PID = PID( 0.3, 0, 0, 1/3)

        self.X_VELOCITY_PID = PID( 1, 0, 0.1, 4)
        self.Y_VELOCITY_PID = PID( 1, 0, 0.1, 4)
        self.Z_VELOCITY_PID = PID( 1, 0, 0.1, 4)

        rospy.Subscriber("/Master/control/error", Float64MultiArray, self.pid_subscriber_callback)
        rospy.Subscriber("mavros/local_position/velocity",  TwistStamped,   self.local_position_velocity_callback)


        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        self.offset_vector = [ 0 , 0 , 0 ]
        self.velocity_vector = TwistStamped

        self.count = 0

        self.main()

    def main(self):
        while True:
            dt = rospy.get_time() - self.old_time
            self.oldtime = rospy.get_time()

            if self.IncomingData[0] == 0:
                x_vel = self.X_VELOCITY_PID(self.IncomingData[3], self.velocity_current.twist.x)
            else:
                x_vel = self.X_POSITION_PID(self.IncomingData[3], 0)

            if self.IncomingData[1] == 0:
                y_vel = self.Y_VELOCITY_PID(self.IncomingData[4], self.velocity_current.twist.y)
            else:
                y_vel = self.Y_POSITION_PID(self.IncomingData[4], 0)

            if self.IncomingData[2] == 0:
                z_vel = self.Z_VELOCITY_PID(self.IncomingData[5], self.velocity_current.twist.z)
            else:
                z_vel = self.Z_POSITION_PID(self.IncomingData[5], 0)



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
            time = rospy.Time.now()
            self.velocity_vector.header.stamp.secs = int(time.to_sec())
            self.velocity_vector.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
            self.velocity_vector.header.seq = self.count

            # use the publisher
            self.velocity_publisher.publish(self.velocity_vector)


##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

    def pid_subscriber_callback(self, IncomingData):
        self.count += 1

        self.IncomingData = IncomingData.data()

##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

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
