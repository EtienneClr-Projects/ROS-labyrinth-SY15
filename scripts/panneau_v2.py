#!/usr/bin/env python3

import numpy as np
import rospy
import time

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

boucle = True


def callback(msg):
    coords = []
    intensities = []

    # besion de faire qu'une boucle, pour envoyer les coordonnées
    global boucle 
    if boucle == False:
        return
    
    boucle = False
    
    # conversion des points en coordonnées (x,y) et suppression des points infini et des points de faible intensité 
    for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        
        if not(msg.ranges[i] == float("inf")):
            if msg.ranges[i] > 0.1 and msg.intensities[i] > 1000:
                coords.append((msg.ranges[i]*np.cos(theta), msg.ranges[i]*np.sin(theta))) 
                intensities.append(msg.intensities[i])
    
    # Create a PointCloud2 (si on souhaite les afficher dans rziv, on peut se servir de pc2)
    pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])

    points = np.array(list(read_points(pc2)))[:,:2]    
    
    # recherche du point à atteindre
    minimum =np.min(points, axis=0)
    maximum =np.max(points, axis=0)

    width = maximum[0] - minimum[0]
    length = maximum[1] - minimum[1]

    center = ((minimum[0] + width/2), minimum[1] + length/2)
    
    premier_point_panneau = points[0]
    dernier_point_panneau = points[-1]
    
    A = dernier_point_panneau[0] - premier_point_panneau[0]
    B = dernier_point_panneau[1] - premier_point_panneau[1]

    point_a_atteindre = (center[0]+A+0.05, center[1]+B+0.05)


    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = "odom"

    goal_pose.pose.position.x = point_a_atteindre[0]
    goal_pose.pose.position.y = point_a_atteindre[1]

    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    goal_publisher.publish(goal_pose)
    
    
    temp = 20
    time.sleep(temp)

    # re-calcul pour avoir une bonne orientation face au panneau 
    point_a_atteindre = (center[0]+A, center[1]+B)

    goal_pose.pose.position.x = point_a_atteindre[0]
    goal_pose.pose.position.y = point_a_atteindre[1]
    
    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    goal_publisher.publish(goal_pose)



if __name__ == '__main__':
    rospy.init_node('transformer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    #rospy.Subscriber('/lidar/points', PointCloud2, callback)

    #pub_pc2 = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()
