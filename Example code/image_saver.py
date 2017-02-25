#!/usr/bin/env python
#__author__ = 'JJ'
import rospy
import cv2
from std_msgs.msg import Bool
camera = cv2.VideoCapture(0)

class TakePicture():
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.framePub = rospy.Publisher('/Vision', Bool, queue_size=10)
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()
            self.framePub.publish(True)

    def main(self):
        picture = self.get_image()
        cv2.imwrite('/home/odroid/catkin_ws/src/iarc_2015/src/iarc_2015/main/mission/vision/frame.jpg',picture)
        cv2.imwrite('/home/odroid/catkin_ws/src/iarc_2015/src/iarc_2015/main/mission/vision/frame2.jpg',picture)


    def get_image(self):
        retval, im = camera.read()
        return im



if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous = True)
    rospy.loginfo('if statement')
    try:
        takepicture = TakePicture()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
