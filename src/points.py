#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion

pub1 = rospy.Publisher('oct1', LaserScan, queue_size=10)
pub2 = rospy.Publisher('oct2', LaserScan, queue_size=10)
pub3 = rospy.Publisher('oct3', LaserScan, queue_size=10)
pub4 = rospy.Publisher('oct4', LaserScan, queue_size=10)
pub5 = rospy.Publisher('oct5', LaserScan, queue_size=10)
pub6 = rospy.Publisher('oct6', LaserScan, queue_size=10)
pub7 = rospy.Publisher('oct7', LaserScan, queue_size=10)
pub8 = rospy.Publisher('oct8', LaserScan, queue_size=10)
pubav = rospy.Publisher('scan_avs', String, queue_size=10)
pubfront = rospy.Publisher('heading_scan', String, queue_size=10)
pubs = [pub1,pub2,pub3,pub4,pub5,pub6,pub7,pub8]
scan = LaserScan()

grid_resolution = 8.0
scan_averages = [0,0,0,0,0,0,0,0]
scan_mins = [0,0,0,0,0,0,0,0]
scan_maxs = [0,0,0,0,0,0,0,0]
heading = ""
shift = {"N":3,"NW":2,"W":1,"SW":0,"S":7,"SE":6,"E":5,"NE":4}

#segments = [-3.14+6.28*(1.0/),-3.14+6.28*(3/16.0),-3.14+6.28*(5/16.0),-3.14+6.28*(7/16.0),-3.14+6.28*(9/16.0),-3.14+6.28*(11/16.0),-3.14+6.28*(13/16.0),-3.14+6.28*(15/16.0)]
segments = []
for i in range(int(grid_resolution)):
    segments.append(-3.14+6.28*((1+i*2)/(grid_resolution*2)))

def rotate(l, n):
        return l[n:] + l[:n]

def _heading_callback(data):
    global heading 
    heading = data.data

def callback(data):
    global scan
    if (heading == ""):
        return -1
    segments_temp = rotate(segments,shift[heading])
    current_time = rospy.Time.now()
    for i in range(int(grid_resolution)):

        scan_step_size = len(data.ranges)/grid_resolution

        scan.header.stamp = current_time
        scan.header.frame_id = data.header.frame_id
        if i<grid_resolution-1:
            scan.angle_min = segments_temp[i]
            scan.angle_max = segments_temp[i+1]
        else:
            scan.angle_min = segments_temp[-1]
            scan.angle_max = segments_temp[0]
        scan.angle_increment = data.angle_increment
        scan.time_increment = data.time_increment
        scan.scan_time = data.scan_time
        scan.range_min = 0.0
        scan.range_max = 100.0

        scan.ranges = []
        scan.intensities = []
        
        temp_data = rotate(data.ranges, int(scan_step_size*shift[heading]))

        if i<grid_resolution-1:
            for j in range(int(i*scan_step_size+(0.5)*scan_step_size),int((i+1)*scan_step_size+(0.5)*scan_step_size)):
                scan.ranges.append(temp_data[j]) 
                #scan.ranges.append(data.ranges[j])  # fake data
                scan.intensities.append(1)  # fake data
            pubs[i].publish(scan)
        else:
            for j in range(int(i*scan_step_size+(0.5)*scan_step_size),int((i+1)*scan_step_size)):
                scan.ranges.append(temp_data[j])
                #scan.ranges.append(data.ranges[j])  # fake data
                scan.intensities.append(1)  # fake data
            for j in range(0,int(0.5*scan_step_size)):
                scan.ranges.append(temp_data[j])
                #scan.ranges.append(data.ranges[j])  # fake data
                scan.intensities.append(1)  # fake data
            pubs[i].publish(scan)
        scan_averages[i] = np.mean(scan.ranges)
        scan_mins[i] = np.min(scan.ranges)
        scan_maxs[i] = np.max(scan.ranges)

    pubav.publish("&".join(map(str,scan_averages)))



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("front/scan", LaserScan, callback)
    rospy.Subscriber("heading", String, _heading_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
