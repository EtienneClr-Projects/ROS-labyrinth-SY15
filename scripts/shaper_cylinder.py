#!/usr/bin/env python3

import numpy as np
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def callback(msg):
    clusters = {}
    cylinders = MarkerArray()
    for x,y,z,c in read_points(msg):
        if c not in clusters.keys(): clusters[c] = []
        clusters[c].append([x,y])

    for c, points in clusters.items():
        points = np.array(points)

        # ToDo: Calculate cluster center
        centre_x = np.mean(points[:,0])
        centre_y = np.mean(points[:,1])
        center = (centre_x, centre_y)
        
        
        # ToDo: Calculate cluster radius
        radius = 0
        for i in range(len(points)):
            radius = max(radius, np.sqrt(((centre_x - points[i][0])**2) + (centre_y - points[i][1])**2))

        if 2*radius<1:
            cylinder = Marker()
            cylinder.header = msg.header
            cylinder.id = c
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position = Point(center[0], center[1], 0)
            cylinder.pose.orientation.w = 1
            cylinder.scale.x, cylinder.scale.y, cylinder.scale.z = 2*radius, 2*radius, 0.3
            cylinder.color.r, cylinder.color.g, cylinder.color.b, cylinder.color.a = 1, 0, 0, 0.5
            cylinder.lifetime = rospy.Duration(0.2)
            cylinders.markers.append(cylinder)

    pub_cylinders.publish(cylinders)

if __name__ == '__main__':
    rospy.init_node('shaper_cylinder')
    pub_cylinders = rospy.Publisher('/lidar/cylinders', MarkerArray, queue_size=10)
    rospy.Subscriber('/lidar/clusters', PointCloud2, callback)
    rospy.spin()
