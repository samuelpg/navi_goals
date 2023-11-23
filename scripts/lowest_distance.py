#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import yaml
import sys

class LowestDistanceCalculator:

    def __init__(self):
        self.data = {"data": []}
        self.name =sys.argv[1]

    def __del__(self):
        with open('{name}.yml'.format(name=self.name), 'w') as outfile:
            yaml.dump(self.data, outfile, default_flow_style=False)

    def callback(self, msg):
        ranges = msg.ranges
        time = msg.header.stamp.secs + (msg.header.stamp.nsecs/1e9)
        
        self.data["data"].append({"d": min(ranges), "t": time})

if __name__ == '__main__':
    rospy.init_node('laser_distance_calculator')
    ldc = LowestDistanceCalculator()
    rospy.Subscriber('/scan_front',LaserScan,ldc.callback) 
    rospy.spin()
