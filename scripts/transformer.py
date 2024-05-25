#!/usr/bin/env python3

import numpy as np
import rospy
import math

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def callback(msg):
    coords = []
    intensities = []
    #print(type(msg))

    for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        
        if not(msg.ranges[i] == float("inf")):
            if msg.ranges[i] > 0.1:
                coords.append((msg.ranges[i]*np.cos(theta), msg.ranges[i]*np.sin(theta))) 
                intensities.append(msg.intensities[i])
    
    # print(coords)
    # Create a PointCloud2 message from coordinates
    pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])
    #pub_pc2.publish(pc2)

    points = np.array(list(read_points(pc2)))[:,:2]
    # print(type(msg))
    # print(points)
    # print(msg.intensities)
    #print(msg.intensities[i] for i in range (len(points)))
    #intensities = np.array(msg.intensities[i] for i in range (len(msg.intensities)))
    #print(len(intensities))
    #intensities = np.array(list(read_points(msg)))[:,:3]
    groups = np.zeros(points.shape[0], dtype=int)

    print("")
    print("")
    #print(points)
    # ToDo: Determine k and D values
    #print("len point")
    #print(len(points))
    k = 10
    D = 0.05
    

    # ToDo: Clustering algorithm
    for i in range(k, points.shape[0]):
        d = []
        for j in range(1, k):
            d.append(np.sqrt(
                    (points[i][0]-points[i-j][0])**2
                    +
                    (points[i][1]-points[i-j][1])**2
                    )
            )   
        dmin= min(d)
        jmin = np.argmin(d)+1
        
        if dmin < D:
            if groups[i-jmin] == 0:
                groups[i-jmin] = max(groups) + 1 
            groups[i]= groups[i-jmin]
            
    #print(len(groups))
    #print(len(points))
    
    i = 0
    groups_ordre = []
    groups_intensity = []

    print(max(groups))
    print(groups_ordre)

    #print("list point")
    #print(points)
    for num_groups in range(max(groups)):
        p_list = []
        i_list= []
        for k in range (len(points)) :
            
            if i == num_groups:
                p_list.append(points[k])
                i_list.append(intensities[k])
        i+=1
        # print(p_list)
        groups_ordre.append(p_list)
        groups_intensity.append(i_list)



    #suppresion des petits groupes
    groups_ordre_tmp = []
    groups_intensity_tmp = []
    for i in range (len(groups_ordre)):
        if len(groups_ordre[i]) > 2:
            groups_ordre_tmp.append(groups_ordre[i])
            groups_intensity_tmp.append(groups_intensity[i])

    groups_ordre = groups_ordre_tmp
    groups_intensity = groups_intensity_tmp

    num_groupe = 0
    
    max_intensity = sum(groups_intensity[0]) / len(groups_intensity[0])

    #recherche de l'intensité la plus élevé
    for i in range (len(groups_ordre)):
        moyenne_intensity = sum(groups_intensity[i]) / len(groups_intensity[i])
        if max_intensity < moyenne_intensity:
            max_intensity = moyenne_intensity
            num_groupe = i
    
    minimum =np.min(groups_ordre[num_groupe], axis=0)
    maximum =np.max(groups_ordre[num_groupe], axis=0)

    width = maximum[0] - minimum[0]
    length = maximum[1] - minimum[1]

    print("largeur")
    print(width)

    print("longueur")
    print(length)
    
    center = ((minimum[0] + width/2), minimum[1] + length/2)
    
    
    premier_point_panneau = groups_ordre[num_groupe][0]
    dernier_point_panneau = groups_ordre[num_groupe][-1]
    
    
    A = dernier_point_panneau[0] - premier_point_panneau[0]
    B = dernier_point_panneau[1] - premier_point_panneau[1]
    
    C = -(A * premier_point_panneau[0] + B * premier_point_panneau[1])

    point_a_atteindre = (center[0]+A, center[1]+B)


    #calcul du point d'arriver du robot 

    theta = np.arctan(point_a_atteindre[1]/point_a_atteindre[0])
    r = center[0] / np.cos(theta)

    print("r avant modification")
    print(r)

    distance_centre_panneau = 0.3
    r = r - distance_centre_panneau


    point_final = r*np.cos(theta), r*np.sin(theta)

    print("point finale")
    print(point_final)

    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = "odom"

    goal_pose.pose.position.x = point_final[0]
    goal_pose.pose.position.y = point_final[1]
    
    dis = math.sqrt((center[0]-premier_point_panneau[0])**2 + (center[1]-premier_point_panneau[1])**2)
    
    theta = math.arctan(dis/distance_centre_panneau)

    goal_pose.pose.orientation.z = math.sin(theta / 2)
    goal_pose.pose.orientation.w = math.cos(theta / 2)
    

    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    goal_publisher.publish(goal_pose)


period = 2

if __name__ == '__main__':
    rospy.init_node('transformer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    #rospy.Subscriber('/lidar/points', PointCloud2, callback)

    #pub_pc2 = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.Timer(rospy.Duration(period), callback)
    rospy.spin()
