#!/usr/bin/env python
#~/Desktop/Projects/llist_Finder1.2 python
#__author__ = 'JJ'
#this program will output to arrarys with horizontal and verticle lines to be used for navigation
#
#Note to self:
#
#check list:
#
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

camera = cv2.VideoCapture(0)  # Turns on camera, sets it to camera. argument is which camera, -1 means to prompt user for camera

class llist_Finder:

    def __init__(self):

        self.Vpub = rospy.Publisher('/Track_line/Vllist', Float64MultiArray, queue_size = 10)
        self.Hpub = rospy.Publisher('/Track_line/Hllist', Float64MultiArray, queue_size = 10)
        self.Angpub = rospy.Publisher('/Track_line/Ang_Hline', Float64, queue_size = 10)

        self.rate = rospy.Rate(10) # 10hz

        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            self.main()
            self.Vpub.publish(self.Vllist)
            self.Hpub.publish(self.Hllist)
            self.Angpub.publish(self.HAngAverage)
            self.rate.sleep()

##############################################################
###################Main Code+ image taking####################
##############################################################
    def main(self):
        self.img = self.get_image()
        self.check = self.get_image()
        global xmax
        global ymax
        xmax, ymax = self.img.shape[:2]  # create the maximum values of the picture
        ###
        self.Hllist = Float64MultiArray()##\\\
        self.Vllist = Float64MultiArray()##---Set up Vllist, Hllist, HAngAverage as data types ros can read
        self.HAngAverage = Float64()#######//

        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY) # converts self.img to a greyscale image. 1st argument is what file to convert.
    	gauss= cv2.GaussianBlur(gray, (9, 9), 0) # Gaussian Blur, no idea what argumets are, copied from Ian's circle detect
        edges = cv2.Canny(gauss, 50, 130, apertureSize=3) #performs canny edge detection on self.img.
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

                    cv2.line(self.check,pt1,pt2,(255,0,0),2)

                    if (100*np.pi/180)> theta > (80*np.pi/180) or (100*np.pi/180)> (theta-np.pi) > (80*np.pi/180):     # horiziontal lines. 80 - 9cx ds0 deg; 260 - 280 deg
                        self.Hllist.data.append((y1+y2)/2) #add the average y point to the list, so that it's in the middle of the image
                        #cv2.line(self.img,pt1,pt2,(0,0,255),2)   #paint the image
                        #print "line in v arrary \n"
                        self.HAngAverage.data += theta*180/np.pi # adds the angle to the angle average

                    elif theta < (10*np.pi/180) or theta > (170*np.pi/180):    #pycharms doesent recognise rospy vertical lines. 10 - 0 deg; 190 - 200 deg
                        self.Vllist.data.append((x1+x2)/2) #add the average x point to the list, so that it's in the middle of the image
                        #cv2.line(self.img,pt1,pt2,(0,255,0),2)   #paint the image
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


        #self.show_images(gray, gauss, edges)
##############################################################
###################End of self.Main Code######################
##############################################################

    def get_image(self):
        retval, im = camera.read()
        return im

    def show_images(self,gray, gauss, edges):

                ### Rounding so that the data type is int. ( don't need this unless your printing the images)
        count = 0
        for i in self.Vllist.data:
            self.Vllist.data[count] = int(round(i))
            count += 1

        count = 0
        for i in self.Hllist.data:
            self.Hllist.data[count] = int(round(i))
            count += 1

        exp = self.getimage()

        for i in self.Hllist.data:
            cv2.line(exp,(0,i),(ymax,i),(0,255,0),2)
    
        for i in self.Vllist.data:
            cv2.line(exp,(i,0),(i,xmax),(0,0,255),2)

        cv2.imshow("1: gray", gray)
        cv2.imshow("2: gauss", gauss)
        cv2.imshow("3: edges", edges)
        cv2.imshow("4: lineimage", self.img)
        cv2.imshow("5: test", exp)
        cv2.imshow("0: check", self.check)

        print "----------------"
        print self.Vllist.data
        print self.Hllist.data
        print self.HAngAverage.data
        cv2.waitKey(50)


##############################################################
###################End of Main Code###########################
##############################################################
if __name__ == '__main__':
	rospy.init_node('llist_finder12', anonymous = True)
	try:
		list = llist_Finder()
	except rospy.ROSInterruptException:
		pass


