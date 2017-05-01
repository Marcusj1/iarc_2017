#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray


#Author: JJ Marcus
#Last Edited Date: 29/10/2016


class LidarSub():

    def __init__(self):
        rospy.loginfo("init")
        #sets the rate of the loop
        self.rate = rospy.Rate(11)
        self.main()


    def main(self):
        print("main")
        self.avoidanglesPublisher = rospy.Publisher("/Angles_to_avoid", Float64MultiArray, queue_size = 10)

        rospy.Subscriber("/scan",LaserScan,self.subscriber)


        while not rospy.is_shutdown():
            self.rate.sleep()



    def subscriber(self, data):
        print("data-------------")
        i = 1
        avoidance_ranges = Float64MultiArray()
        avoidance_angles = Float64MultiArray()
        n = 0
        while i < len(data.ranges) -1:
            if data.ranges[i] < 1.3 and data.ranges[i] > .5: #max and min distance for sight of Lidar
                if data.ranges[i] < data.ranges[i+1] and data.ranges[i] < data.ranges[i-1]:
                    avoidance_ranges.data.append(data.ranges[i])
                    avoidance_angles.data.append(i*0.01745*180/3.1415)
                    n += 1
                    i += 10
            i += 1

        avoidance_ranges.data, avoidance_angles.data = (list(x) for x in zip(*sorted(zip(avoidance_ranges.data, avoidance_angles.data), key = lambda pair: pair[0])))



        print(avoidance_ranges.data)
        print(avoidance_angles.data)

        self.avoidanglesPublisher.publish(avoidance_angles)

if __name__ == '__main__':
    #initate the node
    rospy.init_node('LidarSubscriber')
    try:
        rospy.loginfo("try")
        LidarSub()
    except rospy.ROSInterruptException:
        pass