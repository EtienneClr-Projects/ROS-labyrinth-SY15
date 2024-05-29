#!/usr/bin/env python3

import numpy as np
import rospy
import time
from math import cos, sin, pi

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

class Panneau_Controller():
    def __init__(self) -> None:
        rospy.init_node('transformer')
        rospy.loginfo("Starting panneau node...")

        self.pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
        self.pub_bboxes = rospy.Publisher('/lidar/bboxes', MarkerArray, queue_size=10)
        self.pub_markers = rospy.Publisher('/markers', MarkerArray, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.last_scan = None

        # we sub to /estimation to know when the robot has reached the panel
        rospy.Subscriber('/estimation', PoseWithCovarianceStamped, self.callback_estimation)
        self.pose = None
        
        period = 0.5
        rospy.Timer(rospy.Duration(period), self.callback_timer)

        # self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        # Two states
        # - Going to the front of the panel
        # - Turning the robot to face the panel
        self.state = "GOING_TO_PANEL"
        self.panel_pose = None
        self.panel_reached = False

    def run(self):
        rospy.spin()

    def callback_lidar(self, lidar_msg):
        self.last_scan = lidar_msg

    def callback_estimation(self, estimation_msg):
       # compare current pose to the panel pose
        
        x = estimation_msg.pose.pose.position.x
        y = estimation_msg.pose.pose.position.y
        self.pose = [x,y]

        if self.panel_pose is None:
            return

        if np.sqrt((x - self.panel_pose[0])**2 + (y - self.panel_pose[1])**2) < 0.1:
            self.panel_reached = True



    def publish_goal(self, x, y, theta):
        # goal = PoseStamped()
        # goal.header.frame_id = "odom"
        # goal.header.stamp = rospy.Time.now()
        # goal.pose.position.x = x
        # goal.pose.position.y = y
        # q = quaternion_from_euler(0, 0, theta)
        # goal.pose.orientation = Quaternion(*q)
        
        # print("publishing goal")
        # self.goal_publisher.publish(goal)

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        poses = []
        action = [x,y,theta]
        start = self.pose
        actions = [start, action]

        for action in actions:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            pose.pose.position.x = action[0]
            pose.pose.position.y = action[1]
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            poses.append(pose)
            print(action[0],action[1])

        path_msg.poses = poses
        path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(path_msg)


    def go_to_panel(self):
        if self.last_scan is None or self.pose is None:
            return
        print("in go to panel")
        
        coords = []
        intensities = []
        
        # conversion des points en coordonnées (x,y) et suppression des points infini et des points de faible intensité 
        for i, theta in enumerate(np.arange(self.last_scan.angle_min, self.last_scan.angle_max, self.last_scan.angle_increment)):
            
            if not(self.last_scan.ranges[i] == float("inf")) and self.last_scan.ranges[i] > 0.5:
                if self.last_scan.ranges[i] > 0.1 and self.last_scan.intensities[i] > 5000:
                    coords.append((self.last_scan.ranges[i]*np.cos(theta), self.last_scan.ranges[i]*np.sin(theta))) 
                    intensities.append(self.last_scan.intensities[i])
        
        if not intensities:
            return
        print(max(intensities))
        
        # Create a PointCloud2 (si on souhaite les afficher dans rziv, on peut se servir de pc2)
        #pc2 = create_cloud(self.last_scan.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])

        #points = np.array(list(read_points(pc2)))[:,:2]    
        points = np.array(coords)
        groups = np.zeros(points.shape[0], dtype=int) # liste comprenant le numéro des groupes de chaque point

        k = 10 # nb de point choisie pour faire les clusters 
        D = 0.05 # distance entre les points

        d = np.zeros(k, dtype=float)

        # Clustering algorithm
        for i in range(k, points.shape[0]):
            for j in range(1, k+1):
                d[j-1] = np.linalg.norm(points[i] - points[i-j])
            d_min = np.min(d)
            j_min = np.argmin(d)+1

            if d_min < D:
                if groups[i - j_min] == 0:
                    groups[i - j_min] = max(groups) + 1
                groups[i] = groups[i - j_min]

        print()
        print()
        print()
        print(f"before : {groups}")

        print()
        print()
        print()

        # Delete cluster if too small
        groups_idx, counts = np.unique(groups, return_counts=True)


        print(f"after : {groups_idx}, count : {counts}")
        print()
        print()
        print()
        # for i in range(groups.shape[0]):
        #     if counts[i] < 3:
        #         groups[groups == groups[i]] = 0

        new_points = []

        #groups = [[points[i,0],points[i,1],0,c] for i,c in enumerate(groups)]
        for i, c in enumerate(groups):
            if c == np.argmax(counts):
                new_points.append(points[i])

        points = new_points

        print(f"groups : {groups}")

        print()
        print()
        print()
        print(f"points : {points}")

        print()
        print()
        print()

        # Clustering algorithm
        # for i in range(k, points.shape[0]):
        #     d = []
        #     for j in range(1, k):
        #         d.append(np.sqrt(
        #                 (points[i][0]-points[i-j][0])**2
        #                 +
        #                 (points[i][1]-points[i-j][1])**2
        #                 )
        #         )   
        #     dmin= min(d)
        #     jmin = np.argmin(d)+1
            
        #     if dmin < D:
        #         if groups[i-jmin] == 0:
        #             groups[i-jmin] = max(groups) + 1 
        #         groups[i]= groups[i-jmin]


        # création de plusieurs listes, pour regrouper les points par liste
        # tmp = 0
        # groups_ordre = []
        # groups_intensity = []

        # # à refaire
        # # if not groups.any():
        # #     print(f"ici {groups}")
        # #     return

        # for num_groups in range(max(groups)):
        #     p_list = []
        #     i_list= []
        #     for k in range (len(points)) :
                
        #         if tmp == num_groups:
        #             p_list.append(points[k])
        #             i_list.append(intensities[k])
        #     tmp+=1
        #     groups_ordre.append(p_list)
        #     groups_intensity.append(i_list)

        # groups = [[points[i,0],points[i,1],0,c] for i,c in enumerate(groups)]

        # print(f"group int : {[[points[i,0],points[i,1],0,c] for i,c in enumerate(groups)]}")


        # # suppresion des petits groupes
        # groups_ordre_tmp = []
        # groups_intensity_tmp = []
        # for i in range (len(groups_ordre)):
        #     if len(groups_ordre[i]) > 2:
        #         groups_ordre_tmp.append(groups_ordre[i])
        #         groups_intensity_tmp.append(groups_intensity[i])

        # groups_ordre = groups_ordre_tmp
        # groups_intensity = groups_intensity_tmp

        # num_groupe = 0

        # if not groups_intensity:
        #     return
        
        # max_intensity = sum(groups_intensity[0]) / len(groups_intensity[0])

        # #recherche de l'intensité la plus élevé
        # for i in range (len(groups_ordre)):
        #     moyenne_intensity = sum(groups_intensity[i]) / len(groups_intensity[i])
        #     if max_intensity < moyenne_intensity:
        #         max_intensity = moyenne_intensity
        #         num_groupe = i
        
        # recherche du point à atteindre
        #points = groups_ordre[num_groupe]
        #print(f"group order : {groups_ordre[num_groupe]}")
        minimum =np.min(points, axis=0)
        maximum =np.max(points, axis=0)
        #center = np.median(points,axis=0)
        center = (minimum + maximum)/2
        #width, length = np.max(points-center, axis=0)[0] * 2, np.max(points-center, axis=0)[1] * 2
        #center = ((minimum[0] + width/2), minimum[1] + length/2)
        print("center:",center)
        print(points)
        
        
        premier_point_panneau = np.array(points[0])
        dernier_point_panneau = np.array(points[-1])
 
        vector = dernier_point_panneau - premier_point_panneau
        vector = vector/np.linalg.norm(vector)
        # vector = np.array([
        #     vector[0]*cos(-pi/2) - vector[1]*sin(-pi/2),
        #     vector[0]*sin(-pi/2) + vector[1]*cos(-pi/2)
        # ])
        vector *= -0.2
        print(vector)

        point_a_atteindre = center + vector
        #pc2 = create_cloud(self.last_scan.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])
        #self.publish_goal(center[], center, 0)
        self.publish_goal(center[0], center[1], 0)

            # Calculate cluster length and width
        width, length = np.max(points-center, axis=0)[0] * 2, np.max(points-center, axis=0)[1] * 2

        bboxes = MarkerArray()

        bbox = Marker()
        bbox.header = self.last_scan.header
        bbox.id = 1
        bbox.type = Marker.CUBE
        bbox.action = Marker.ADD
        bbox.pose.position = Point(center[0], center[1], 0)
        bbox.pose.orientation.w = 1
        bbox.scale.x, bbox.scale.y, bbox.scale.z = width, length, 0.3
        bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1, 0, 0, 0.5
        bboxes.markers.append(bbox)
        self.pub_bboxes.publish(bboxes)

        bboxes = MarkerArray()

        for i, point in enumerate(points):
            bbox = Marker()
            bbox.header = self.last_scan.header
            bbox.id = i
            bbox.type = Marker.SPHERE
            bbox.action = Marker.ADD
            bbox.pose.position = Point(point[0], point[1], 0)
            bbox.pose.orientation.w = 1
            bbox.scale.x, bbox.scale.y, bbox.scale.z = 0.1, 0.1, 0.3
            bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1, 0, 0, 0.5
            bboxes.markers.append(bbox)
        self.pub_markers.publish(bboxes)

        # self.state = "TURNING_ROBOT"
        self.state = ""
        self.panel_pose = point_a_atteindre

    def turn_robot(self):
        pass
        # # re-calcul pour avoir une bonne orientation face au panneau 
        # goal_pose = PoseStamped()
        # goal_pose.pose.position.x = self.panel_pose[0]
        # goal_pose.pose.position.y = self.panel_pose[1]

        # # orientation du robot
        # # TODO, et coder un turn robot dans le controller ?
        
        # goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        # goal_publisher.publish(goal_pose)
        # print("published goal")

    def callback_timer(self, _):
        # print(self.state)
        if self.state == "END":
            return
        
        if self.state == "GOING_TO_PANEL":
            self.go_to_panel()
            print("Robot en route vers le panneau")

        elif self.state == "TURNING_ROBOT":
            if not self.panel_reached:
                return
            self.turn_robot()
            self.state = "END"



if __name__ == '__main__':
    panneau = Panneau_Controller()
    panneau.run()