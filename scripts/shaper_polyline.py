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

# Computes the distance from point pM to segment [p1,p2]
def dist_seg(pM, p1, p2):
    vec = p2 - p1
    vec /= np.linalg.norm(vec)
    vecT = np.array([-vec[1], vec[0]])
    return np.abs(np.sum((pM-p1)*vecT))

def rdp(points, eps):
    # ToDo: Ramer-Douglas-Peucker Algorithm
    dmax = 0
    jmax = 0
    d = np.zeros(len(points))
    
#    print(len(points))
    for i in range(0, len(points) - 1):
        
#        print(points[-1])
#        print(points[0])
#        print(i)
        print(points[i])
        print("")
        d[i] = dist_seg(points[i], points[0], points[-1])
    dmax= np.max(d)
    print(d)
    jmax = int(np.where(d == dmax)[0][0])
#    print(jmax)
    
    if dmax > eps : 
        pres1 = rdp(points[:jmax], eps)
        pres2 = rdp(points[jmax:], eps)
        res_points = np.concatenate((pres1[:len(pres1)-1], pres2))
    else :
        res_points = [points[0], points[-1]]
    
#    print(res_points)
    return res_points


def callback(msg, eps):
    clusters = {}
    polylines = MarkerArray()
    for x,y,z,c in read_points(msg):
        if c not in clusters.keys(): clusters[c] = []
        clusters[c].append([x,y])
        
    
#    clusters2 = {}
#    j = 0
#    for i in range(1, len(clusters)):
#        if len(clusters[i]) > 2:
#            if j not in clusters2.keys(): clusters2[j] = []
#            clusters2[j].append(clusters[i])
#            j += 1
            
#    print(clusters)
#    print(clusters2)

#    clusters = clusters2
    
    for c, points in clusters.items():
        poly = rdp(np.array(points), eps)

        polyline = Marker()
        polyline.header = msg.header
        polyline.id = c
        polyline.type = Marker.LINE_STRIP
        polyline.action = Marker.ADD
        polyline.pose.orientation.w = 1
        polyline.scale.x, polyline.scale.y, polyline.scale.z = eps*2, eps*2, 0.3
        polyline.color.r, polyline.color.g, polyline.color.b, polyline.color.a = 1, 0, 0, 0.5
        polyline.lifetime = rospy.Duration(0.2)
        for x,y in poly:
            point = Point()
            point.x, point.y, point.z = x, y, 0
            polyline.points.append(point)
        polylines.markers.append(polyline)

    pub_polylines.publish(polylines)


    #calcul de la moyenne avec une forte intensité 
    

if __name__ == '__main__':
    rospy.init_node('shaper_polyline')
    eps = rospy.get_param('~eps', 0.05)
    pub_polylines = rospy.Publisher('/lidar/polylines', MarkerArray, queue_size=10)
    rospy.Subscriber('/lidar/clusters', PointCloud2, callback, eps)
    rospy.spin()
