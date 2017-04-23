#!/usr/bin/env python
#
# Project: Video Streaming with Flask
# Author: Log0 <im [dot] ckieric [at] gmail [dot] com>
# Date: 2014/12/21
# Website: http://www.chioka.in/
# Description:
# Modified to support streaming out with webcams, and not just raw JPEGs.
# Most of the code credits to Miguel Grinberg, except that I made a small tweak. Thanks!
# Credits: http://blog.miguelgrinberg.com/post/video-streaming-with-flask
#
# Usage:
# 1. Install Python dependencies: cv2, flask. (wish that pip install works like a charm)
# 2. Run "python main.py".
# 3. Navigate the browser to the local webpage.
import rospy
import cv2
from flask import Flask, render_template, Response
from sensor_msgs.msg import *
from cv_bridge import CvBridge
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    print('video_feed function')
    return Response(image_grabber.notaninit(),mimetype='multipart/x-mixed-replace; boundary=frame')

class image_grabber:

    def __init__(self):
        rospy.Subscriber("/mavros/state",Image,self.image_callback)

        app.run(host='0.0.0.0', debug=True)
    def image_callback(self, circles):
        print('image_callback')
        bridge = CvBridge()
        self.circles = bridge.imgmsg_to_cv2(circles, "bgr8")
    def notaninit(self):
        print("init function")
        print('does it get here?')
        while True:
            print('main loop')
            jpeg = cv2.imencode('.jpg', self.circles)
            print('imencode to jpg?')
            frame = jpeg.tostring()
            print('onto the yield')
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

if __name__ == '__main__':
    rospy.init_node('webhost', anonymous=True)
    print("run")

    try:
        grab = image_grabber()
    except rospy.ROSInterruptException:
        pass
