#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid



def callback(data):
    map_data = np.array(data.data)
    unknowns = np.count_nonzero(map_data == -1)
    information_metric = len(map_data)-unknowns
    print(information_metric)
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/map", OccupancyGrid, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
