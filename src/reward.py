#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid


class Map_metric:
    information_metric = 0
    old_info_metric = 0

    def callback(self, data):
        map_data = np.array(data.data)
        unknowns = np.count_nonzero(map_data == -1)
        self.information_metric = len(map_data)-unknowns
        #print(self.information_metric)

    def delta_score(self):
        value = self.information_metric - self.old_info_metric
        self.old_info_metric = self.information_metric
        return value
    

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

if __name__ == '__main__':
    map_object = Map_metric()
    map_object.listener()
    while not rospy.is_shutdown():
        value = map_object.delta_score()
        if value>0:
            print(value)