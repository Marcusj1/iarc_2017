#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from std_msgs.msg import *
import math

# Edited on Dec 1 2016 by JJ
# fixed altitude not being a float issue


class CircleDetect():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/roomba/location_meters', PoseArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        # For finding radius
        self.a = 391.3
        self.b = -0.9371
        self.radius = 0
        # Altitude
        self.alt = 0.8128 # 32 inches ~ as high as the countertop  # NEEDS TO CHANGE FROM HARD CODE TO SUBSCRIPTION
        #self.alt = 0.3048 # 12 inches ~ as high as the countertop
        #self.alt = 1.143 # 45 inches ~ as high as the countertop
        self.pose_array = PoseArray()
        self.temp_pose = Pose()
        self.header = Header()
        self.header.seq = 0
        self.header.stamp = rospy.Time.now()
        self.cap = cv2.VideoCapture(0)
        self.loop_search()

    def loop_search(self):
        while not rospy.is_shutdown():
            # Uncomment for actual flight to find altitude
            #rospy.Subscriber("mavros/altitude",Altitude,self.altitude_callback)

            # read frame from capture
            ret, img = self.cap.read()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            output = img.copy()

            # Gaussian Blur
            gaussian_blur = cv2.GaussianBlur(gray, (9, 9), 0)
            # Get the radius range based off of altitude (alt in meter, radius in pixel)
            self.radius = int(-12.1 * math.pow(self.alt, 4) +
            49.188 * math.pow(self.alt, 3) - 21.3 * math.pow(self.alt, 2)
            - 132.26 * self.alt + 176.6)

            circles = cv2.HoughCircles(gaussian_blur, cv2.cv.CV_HOUGH_GRADIENT,
                3, 100, minRadius=self.radius - 5, maxRadius=self.radius + 5)

            # ensure at least some circles were found
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                self.pose_array = []
                for (x, y, r) in circles:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5),
                        (0, 128, 255), -1)
                    # TO find the length on the ground in meters
                    # (height in meters times the distance in pixels)/360
                    self.temp_pose.position.x = (( x- 320) * self.alt) / 360
                    self.temp_pose.position.y = ((240 - y) * self.alt) / 360
                    # Published the pixel location as well
                    self.temp_pose.orientation.x = ( x- 320)
                    self.temp_pose.orientation.y = (240 - y)
                    self.temp_pose.position.z = 0
                    self.pose_array.append(self.temp_pose)

            self.header.seq += 1
            #self.header.stamp = rospy.get_time()
            time = rospy.Time.now()
            self.header.stamp.secs = int(time.to_sec())
            self.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
            roomba_locations = PoseArray( self.header,self.pose_array)
            print(roomba_locations)

            #########################
            # show the output image #
            #########################
            cv2.imshow("output", np.hstack([img, output]))
            # exit condition to leave the loop
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break

            ###############################################################
            ##############################Publisher########################
            ###############################################################
            self.pub.publish(roomba_locations)  # PoseArray type variable
            rospy.loginfo (roomba_locations)


            #self.rospy.spin()
            self.rate.sleep()

        cv2.destroyAllWindows()
        self.cap.release()

    def altitude_callback(self, data):
        IncomingData = [0,0,0,0,0]
        IncomingData[0] = data.monotonic
        IncomingData[1] = data.amsl
        IncomingData[2] = data.relative
        IncomingData[3] = data.terrain
        IncomingData[4] = data.bottom_clearance

        if IncomingData[2] > 0.3 and IncomingData[2] < 20:
            self.alt = ( IncomingData[2] )

        else:
            self.alt = ( IncomingData[1] - IncomingData[3] )

        return

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Circle_Detect', anonymous=True)
    try:
        circle = CircleDetect()
    except rospy.ROSInterruptException:
        pass
