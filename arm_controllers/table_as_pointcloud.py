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

    d = 0.4/49
    d = np.float32(d)

    dz = 0.05/49;
    dz = np.float32(dz)
    particles = np.empty((125000,3))
    particles_x = np.linspace(0.0,0.4)
    particles_y = np.linspace(-0.7,0.-0.3)
    particles_z = np.linspace(0.2,0.25)

    x,y,z = np.meshgrid(particles_x, particles_y, particles_z, indexing='xy')

    particles[:,0] = np.ravel(x)
    particles[:,1] = np.ravel(y)
    particles[:,2] = np.ravel(z)

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