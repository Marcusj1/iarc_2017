#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *

#Author: JJ Marcus
#Last Edited Date: 29/10/2016


class Listener():

    def __init__(self):
        rospy.loginfo("init")
        #sets the rate of the loop
        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.main()


    def main(self):
        rospy.loginfo('main')
        rospy.Subscriber("/mavros/Altitude/",Float64,self.print_info_callback)
        self.rate.sleep()


    def print_info_callback(self, data):
        rospy.loginfo("Log:")
        rospy.loginfo(str(data))

if __name__ == '__main__':
    #initate the node
    rospy.init_node('SimpleAltSubscriber')
    try:
        rospy.loginfo("try")
        Listener()
    except rospy.ROSInterruptException:
        pass