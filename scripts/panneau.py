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
    
    # besion de faire qu'une boucle, pour envoyer les coordonnées
    global boucle 
    if boucle == False:
        return
    
    boucle = False
    
    coords = []
    intensities = []

    # conversion des points en coordonnées (x,y) et suppression des points infini
    for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        
        if not(msg.ranges[i] == float("inf")):
            if msg.ranges[i] > 0.1:
                coords.append((msg.ranges[i]*np.cos(theta), msg.ranges[i]*np.sin(theta))) 
                intensities.append(msg.intensities[i])
    
    # Create a PointCloud2 (si on souhaite les afficher dans rziv, on peut se servir de pc2)
    pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])

    points = np.array(list(read_points(pc2)))[:,:2]
    groups = np.zeros(points.shape[0], dtype=int) # liste comprenant le numéro des groupes de chaque point 
    

    k = 10 # nb de point choisie pour faire les clusters 
    D = 0.05 # distance entre les points

    # Clustering algorithm
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


    # création de plusieurs listes, pour regrouper les points par liste
    tmp = 0
    groups_ordre = []
    groups_intensity = []

    for num_groups in range(max(groups)):
        p_list = []
        i_list= []
        for k in range (len(points)) :
            
            if tmp == num_groups:
                p_list.append(points[k])
                i_list.append(intensities[k])
        tmp+=1
        groups_ordre.append(p_list)
        groups_intensity.append(i_list)


    # suppresion des petits groupes
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
    
    # recherche du point à atteindre
    minimum =np.min(groups_ordre[num_groupe], axis=0)
    maximum =np.max(groups_ordre[num_groupe], axis=0)

    width = maximum[0] - minimum[0]
    length = maximum[1] - minimum[1]

    center = ((minimum[0] + width/2), minimum[1] + length/2)
    
    premier_point_panneau = groups_ordre[num_groupe][0]
    dernier_point_panneau = groups_ordre[num_groupe][-1]
    
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
    # je ne sais pas comment obtenir l'angle de fin donc je triche un peu en donnant un point à atteindre sur le vecteur normal
    # au pannneau et ensuite je met le vrai point à atteindre donc l'orientation doit être bonne 

    point_a_atteindre = (center[0]+A, center[1]+B)

    goal_pose.pose.position.x = point_a_atteindre[0]
    goal_pose.pose.position.y = point_a_atteindre[1]
    
    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    goal_publisher.publish(goal_pose)



if __name__ == '__main__':
    rospy.init_node('transformer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()