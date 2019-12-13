#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion

# octal segment publishers
pub1 = rospy.Publisher('oct1', LaserScan, queue_size=10)
pub2 = rospy.Publisher('oct2', LaserScan, queue_size=10)
pub3 = rospy.Publisher('oct3', LaserScan, queue_size=10)
pub4 = rospy.Publisher('oct4', LaserScan, queue_size=10)
pub5 = rospy.Publisher('oct5', LaserScan, queue_size=10)
pub6 = rospy.Publisher('oct6', LaserScan, queue_size=10)
pub7 = rospy.Publisher('oct7', LaserScan, queue_size=10)
pub8 = rospy.Publisher('oct8', LaserScan, queue_size=10)
# publisher list to be iterated through
pubs = [pub1,pub2,pub3,pub4,pub5,pub6,pub7,pub8]

# stat publishers
pubav = rospy.Publisher('scan_avs', String, queue_size=10)
pubfront = rospy.Publisher('heading_scan', LaserScan, queue_size=10)

# Laserscan object to be filled and published in segments
scan = LaserScan()

# grid resolution <float> (ability to change is not implemented yet) 
grid_resolution = 8.0

# Placeholder data structures
scan_averages = [0,0,0,0,0,0,0,0]
scan_mins = [0,0,0,0,0,0,0,0]
scan_maxs = [0,0,0,0,0,0,0,0]
heading = ""

# Dict used to shift segments so that they are in the glabal world frame
shift = {"N":3,"NW":2,"W":1,"SW":0,"S":7,"SE":6,"E":5,"NE":4}
# Dict used to determine which segment is the front of the robot
forward_shift = {"N":0,"NW":1,"W":2,"SW":3,"S":4,"SE":5,"E":6,"NE":7}


# Segment min and max angle value calculation
segments = []
for i in range(int(grid_resolution)):
    segments.append(-3.14+6.28*((1+i*2)/(grid_resolution*2)))

# function to help shift lists
def rotate(l, n):
    """ Function to rotate entries in list "l" by n positions

        Args:
            l (list): list to be rotated
            n (int): positions to rotate by

        Returns:
            new_l (list): the rotated list

    """
        return l[n:] + l[:n]

def _heading_callback(data):
    """ Callback to get heading direction of robot and store it

        Args:
            data: ros message of String

        Returns:
            None

        """
    global heading 
    heading = data.data

def _laser_callback(data):
    """ Callback to laserscan data and segment it into 8 equal segments
        The callback also publishes these 8 segments along with the averages
        of each segment, and publishes the segment corresponding to the front
        of the robot to its own topci

        Args:
            data: ros message of type nav_msgs/Odometry

        Returns:
            None

        """
    global scan
    # Check we have a heading
    if (heading == ""):
        return -1

    # Rotate segments to match co-ordinate system
    segments_temp = rotate(segments,shift[heading])
    current_time = rospy.Time.now()

    for i in range(int(grid_resolution)):
        # Calculate how many points in each segment
        scan_step_size = len(data.ranges)/grid_resolution

        # Fill laser scan header
        scan.header.stamp = current_time
        scan.header.frame_id = data.header.frame_id

        # Fill in scan min and max angles for each segment
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

        # Clear data from scan message
        scan.ranges = []
        scan.intensities = []

        # shift data so that octals always match same global position
        temp_data = rotate(data.ranges, int(scan_step_size*shift[heading]))

        # Fill in range data for each segment
        if i<grid_resolution-1:
            for j in range(int(i*scan_step_size+(0.5)*scan_step_size),int((i+1)*scan_step_size+(0.5)*scan_step_size)):
                scan.ranges.append(temp_data[j]) 
                #scan.ranges.append(data.ranges[j])
                scan.intensities.append(1)  # fake data
            pubs[i].publish(scan)
        else:
            for j in range(int(i*scan_step_size+(0.5)*scan_step_size),int((i+1)*scan_step_size)):
                scan.ranges.append(temp_data[j])
                #scan.ranges.append(data.ranges[j])
                scan.intensities.append(1)  # fake data
            for j in range(0,int(0.5*scan_step_size)):
                scan.ranges.append(temp_data[j])
                #scan.ranges.append(data.ranges[j])
                scan.intensities.append(1)  # fake data
            pubs[i].publish(scan)

        # Calculate metrics on each segment
        scan_averages[i] = np.mean(scan.ranges)
        scan_mins[i] = np.min(scan.ranges)
        scan_maxs[i] = np.max(scan.ranges)

        # Publish the laser scan data from the front of the robot
        if (i == forward_shift[heading]):
            pubfront.publish(scan)
    pubav.publish("&".join(map(str,scan_averages)))



def listener():
    # init node 
    rospy.init_node('point_publisher', anonymous=True)

    rospy.Subscriber("front/scan", LaserScan, _laser_callback)
    rospy.Subscriber("heading", String, _heading_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
