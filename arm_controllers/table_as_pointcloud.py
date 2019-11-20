import numpy as np
from numpy.random import uniform

#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
from rospy import Time


def table_points():
    pub = rospy.Publisher('particles_pcl', PointCloud2, queue_size = 10)
    rospy.init_node('table_as_points', anonymous=True)

    rate = rospy.Rate(10)
    particles = np.empty((500, 3))
    particles[:, 0] = uniform(0.2, 0.6, 500)
    particles[:, 1] = uniform(0.2, 0.6, 500)
    particles[:, 2] = 0.1
    particles = particles.tolist()
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        pointcloud = pcl2.create_cloud_xyz32(header,particles)

        pub.publish(pointcloud)
        rate.sleep()


if __name__ == '__main__':
    table_points()
