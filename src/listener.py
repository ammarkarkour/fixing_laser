#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class FixedLaserScan(object):
    def __init__(self, pub):
        self._pub = pub
        
    def callback(self, msg):
        
        scan = LaserScan()

        scan.header.seq = msg.header.seq
        scan.header.stamp.secs = msg.header.stamp.secs
        scan.header.stamp.nsecs = msg.header.stamp.nsecs
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max     
 
        scan.intensities = msg.intensities

        num_readings = len(msg.ranges)
        scan.ranges = list()
        for i in range(0, num_readings):
            
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
#                print("u[pdated something")
                scan.ranges.append(float("inf"))
            else:
                scan.ranges.append(msg.ranges[i])
 #           scan.ranges[i] = float("inf")

        self._pub.publish(scan)
        print("published")
                

def listener():

    rospy.init_node('listener', anonymous=True)   
    pub = rospy.Publisher("scan2", LaserScan, queue_size = 50)
    fix = FixedLaserScan(pub)

    rospy.Subscriber("/scan", LaserScan, fix.callback)
    rospy.spin()
   
if __name__ == '__main__':
    listener()
