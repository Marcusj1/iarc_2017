#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from std_msgs.msg import *
from sensor_msgs.msg import *
import math
import time

# Edited on Dec 1 2016 by JJ
# fixed altitude not being a float issue


class CircleDetect():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/Vison/roomba_location_meters', PoseArray, queue_size=10)
        self.rate = rospy.Rate(10)  # needs to be runing fairly fast
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
        self.roomba_array = PoseArray()

        #rospy.Subscriber("mavros/altitude",Altitude,self.altitude_callback)
        rospy.Subscriber("mavros/px4flow/ground_distance",  Range,   self.flownar)


        self.Vpub = rospy.Publisher('/Vision/Vllist', Float64MultiArray, queue_size = 10)
        self.Hpub = rospy.Publisher('/Vision/Hllist', Float64MultiArray, queue_size = 10)
        self.Angpub = rospy.Publisher('/Vision/Ang_Hline', Float64, queue_size = 10)


        self.loop_search()

    def loop_search(self):
        while not rospy.is_shutdown():
            # read frame from capture
            ret, img = self.cap.read()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            output = img.copy()

            # Gaussian Blur
            gaussian_blur = cv2.GaussianBlur(gray, (9, 9), 0)
            # Get the radius range based off of altitude (alt in meter, radius in pixel)
            if self.alt < 0:
                self.alt = 0
            self.radius = int(-12.1 * math.pow(self.alt, 4) +
            49.188 * math.pow(self.alt, 3) - 21.3 * math.pow(self.alt, 2)
            - 132.26 * self.alt + 176.6)

            circles = cv2.HoughCircles(gaussian_blur, cv2.cv.CV_HOUGH_GRADIENT,
                    3, 100, minRadius=self.radius - 5, maxRadius=self.radius + 5)

            self.pose_array = []



            # ensure at least some circles were found
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                for (x, y, r) in circles:

                    # for visulization:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                    # TO find the length on the ground in meters
                    # (height in meters times the distance in pixels)/360
                    self.temp_pose.position.x = (( x- 320) * self.alt) / 360
                    self.temp_pose.position.y = ((240 - y) * self.alt) / 360
                    # Published the pixel location as well
                    self.temp_pose.orientation.x = ( x- 320)
                    self.temp_pose.orientation.y = (240 - y)
                    self.temp_pose.position.z = 0
                    self.pose_array.append(self.temp_pose)
                    print( [self.temp_pose.position.x, self.temp_pose.position.y] )

            self.roomba_array.header.seq += 1
            time = rospy.Time.now()
            self.roomba_array.header.stamp.secs = int(time.to_sec())
            self.roomba_array.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
            self.roomba_array.poses = self.pose_array

            ################################################
            ################ Line Detection ################
            ################################################

            global xmax
            global ymax
            xmax, ymax = img.shape[:2]  # create the maximum values of the picture
            ###
            self.Hllist = Float64MultiArray()##\\\
            self.Vllist = Float64MultiArray()##---Set up Vllist, Hllist, HAngAverage as data types ros can read
            self.HAngAverage = Float64()#######//

            edges = cv2.Canny(gaussian_blur, 50, 130, apertureSize=3) #performs canny edge detection on self.img.
            # arguments: (image file to perform, minVal [below this are surely not edges], maxVal [above this are sure to be edges],
            # aperture size [default 3],L2gradient[default is true, decides which equn to use for finding the graident])
            ################################################
            lines = cv2.HoughLines(edges, 1, np.pi/180, 120)
            ################################################
            ##### makes lines. args:(image to work with, length, angular resolution, maximum gap between lines)
            ##### outputs an angle and a length. Angle is angle down from the horizontal, length is length from the origin (center of image)
            ##### 0 < angle < 180, length can be negathLinesive.

            if lines is not None:   #if there are not no lines
                   for rho, theta in lines[0]:
                       ##### \/\/\/\/ ####
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho

                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))

                        pt1 = (x1,y1)
                        pt2 = (x2,y2)
                        #### /\/\/\ #### all stuff on the open.cv website

                        #print rho, "\t\t'", theta*180/np.pi, "\n",

                        #cv2.line(self.check,pt1,pt2,(255,0,0),2)

                        if (100*np.pi/180)> theta > (80*np.pi/180) or (100*np.pi/180)> (theta-np.pi) > (80*np.pi/180):     # horiziontal lines. 80 - 9cx ds0 deg; 260 - 280 deg
                            self.Hllist.data.append((y1+y2)/2) #add the average y point to the list, so that it's in the middle of the image
                            #cv2.line(output,pt1,pt2,(0,0,255),2)   #paint the image
                            #print "line in v arrary \n"
                            self.HAngAverage.data += theta*180/np.pi # adds the angle to the angle average

                        elif theta < (10*np.pi/180) or theta > (170*np.pi/180):    #pycharms doesent recognise rospy vertical lines. 10 - 0 deg; 190 - 200 deg
                            self.Vllist.data.append((x1+x2)/2) #add the average x point to the list, so that it's in the middle of the image
                            #cv2.line(output,pt1,pt2,(0,255,0),2)   #paint the image
                            #print "line is in h arrary \n"

            ##############################################################
            ###################Processing The arrays######################
            ##############################################################
            self.Hllist.data.sort() # sorts arrarys
            self.Vllist.data.sort() # /\

            ### average the angle
            if len(self.Hllist.data) != 0: # if the num of elements in Hllist is not 0, then:
                self.HAngAverage.data = self.HAngAverage.data/len(self.Hllist.data) #devide angle by num of elements (sum/number) (average)
            else:
                self.HAngAverage.data = 999 # if there are no elements in Hllist, then no lines, so the error angle is 999
            ###


            #########################
            # show the output image #
            #########################

            #for visulization:
            #cv2.imshow("output", np.hstack([img, output]))

            #exit condition to leave the loop
            k = cv2.waitKey(30) & 0xff
            if k == 27:
               break

            ###############################################################
            ##############################Publisher########################
            ###############################################################
            self.pub.publish(self.roomba_array)  # PoseArray type variable
            self.Vpub.publish(self.Vllist)
            self.Hpub.publish(self.Hllist)
            self.Angpub.publish(self.HAngAverage)

            print('=============================================')

            print(self.Vllist.data)
            print(self.Hllist.data)
            print(self.HAngAverage.data)


            #self.rospy.spin()
            self.rate.sleep()

        cv2.destroyAllWindows()
        self.cap.release()

    ###################################
    def flownar(self, IncomingData):

        if not IncomingData.range == 0:
            self.alt = IncomingData.range -0.05
            #print("-----------------------------------"+str(self.alt))
    ###################################

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('roomba', anonymous=True)
    try:
        circle = CircleDetect()
    except rospy.ROSInterruptException:
        pass
