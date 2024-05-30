#!/usr/bin/env python3

import numpy as np
import rospy
import time
from math import cos, sin, pi, inf

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



    def publish_path(self, points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        poses = []
        for action in points:
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
        print("TRYING TO FIND THE PANEL")
        
        coords = []
        intensities = []
        
        # conversion des points en coordonnées (x,y) et suppression des points infini et des points de faible intensité 
        for i, theta in enumerate(np.arange(self.last_scan.angle_min, self.last_scan.angle_max, self.last_scan.angle_increment)):
            
            if not(self.last_scan.ranges[i] == float("inf")) and self.last_scan.ranges[i] > 0.5:
                if self.last_scan.ranges[i] > 0.1 and self.last_scan.intensities[i] > 5000:
                    coords.append((self.last_scan.ranges[i]*np.cos(theta), self.last_scan.ranges[i]*np.sin(theta))) 
                    intensities.append(self.last_scan.intensities[i])
        
        # sécurité
        if not intensities:
            return
        
        k = 2 # nb de point choisie pour faire les clusters 
        D = 0.03 # distance entre les points

        d = np.zeros(k, dtype=float)
        points = np.array(coords) # liste des points
        groups_idx = np.zeros(points.shape[0], dtype=int) # liste comprenant le numéro des groupes de chaque point

        # Clustering algorithm
        for i in range(k, points.shape[0]):
            for j in range(1, k + 1):
                d[j - 1] = np.linalg.norm(points[i] - points[i - j])
            d_min = np.min(d)
            j_min = np.argmin(d) + 1

            if d_min < D:
                if groups_idx[i - j_min] == 0:
                    groups_idx[i - j_min] = max(groups_idx) + 1
                groups_idx[i] = groups_idx[i - j_min]
            else:
                groups_idx[i] = max(groups_idx) + 1

        # recherche du cluster avec le plus de points
        _, counts = np.unique(groups_idx, return_counts=True)
        max_count_idx = np.argmax(counts)
        points = [[point[0], point[1]] for idx, point in zip(groups_idx, points) if idx == max_count_idx]

        # points = [[1,1],[1,1.3],[1,1.4],[1,1.8]] tests datasets
        # points = [[1,1],[1.2,1],[1.4,1],[1.8,1]]

        print("FOUND THE PANEL")
        
        min_x = inf
        max_x = -inf
        minimum = None
        maximum = None
        for p in points:
            if p[0]<min_x:
                min_x=p[0]
                minimum=p
            if p[0]>max_x:
                max_x=p[0]
                maximum=p

        if abs(min_x-max_x)<0.2:
            # on regarde en y du coup car les x sont trop proches
            min_y = inf
            max_y = -inf
            minimum = None
            maximum = None
            for p in points:
                if p[1]<min_y:
                    min_y=p[1]
                    minimum=p
                if p[1]>max_y:
                    max_y=p[1]
                    maximum=p 

        minimum = np.array(minimum)
        maximum = np.array(maximum)


        # récupération des extrémités du cluster (comparaison sur l'axe X uniquement)
        # recherche du point à atteindre
        center = (minimum + maximum)/2
        print("center:",center)
        points_to_display = [minimum, maximum, center] #DEBUG
        print(points_to_display)
 
        vector = maximum - minimum
        vector_unit = vector/np.linalg.norm(vector)
        vector_unit = np.array([ #turn 90°
            vector_unit[0]*cos(-pi/2) - vector_unit[1]*sin(-pi/2),
            vector_unit[0]*sin(-pi/2) + vector_unit[1]*cos(-pi/2)
        ])

        point_a_atteindre1 = center + vector_unit*0.5
        point_a_atteindre2 = center - vector_unit*0.5

        # en fonction des points, point à atteindre est du bon côté du panneau ou pas
        # on prend le plus proche
        if np.linalg.norm(self.pose-point_a_atteindre1) < np.linalg.norm(self.pose-point_a_atteindre2):
            point_a_atteindre = point_a_atteindre1
            point2 = center+vector_unit*(0.5-0.2)
        else:
            point_a_atteindre = point_a_atteindre2
            point2 = center-vector_unit*(0.5-0.2)


        start = [self.pose[0],self.pose[1],0.]
        point1 = [point_a_atteindre[0],point_a_atteindre[1],0.]
        point2 = [point2[0],point2[1],0.]
        path = [start,point1,point2]

        self.publish_path(path)
        

        # affiche les points to display
        bboxes = MarkerArray()
        for i, point in enumerate(points_to_display):
            bbox = Marker()
            bbox.header = self.last_scan.header
            bbox.id = i
            bbox.type = Marker.SPHERE
            bbox.action = Marker.ADD
            bbox.pose.position = Point(point[0], point[1], 0)
            bbox.pose.orientation.w = 1
            bbox.scale.x, bbox.scale.y, bbox.scale.z = 0.1, 0.1, 0.3
            bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 0, 1, 0, 0.5
            bboxes.markers.append(bbox)
        self.pub_markers.publish(bboxes)

        # Calculate cluster length and width
        #width, length = np.max(points-center, axis=0)[0] * 2, np.max(points-center, axis=0)[1] * 2

        # bboxes = MarkerArray()
        # bbox = Marker()
        # bbox.header = self.last_scan.header
        # bbox.id = 1
        # bbox.type = Marker.CUBE
        # bbox.action = Marker.ADD
        # bbox.pose.position = Point(center[0], center[1], 0)
        # bbox.pose.orientation.w = 1
        # bbox.scale.x, bbox.scale.y, bbox.scale.z = width, length, 0.3
        # bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1, 0, 0, 0.5
        # bboxes.markers.append(bbox)
        # self.pub_bboxes.publish(bboxes)
        

        # bboxes = MarkerArray()
        # for i, point in enumerate(points):
        #     bbox = Marker()
        #     bbox.header = self.last_scan.header
        #     bbox.id = i
        #     bbox.type = Marker.SPHERE
        #     bbox.action = Marker.ADD
        #     bbox.pose.position = Point(point[0], point[1], 0)
        #     bbox.pose.orientation.w = 1
        #     bbox.scale.x, bbox.scale.y, bbox.scale.z = 0.1, 0.1, 0.3
        #     bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1, 0, 0, 0.5
        #     bboxes.markers.append(bbox)
        # self.pub_markers.publish(bboxes)

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