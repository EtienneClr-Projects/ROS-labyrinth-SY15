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

class Panneau_Controller():
    def __init__(self) -> None:
        rospy.init_node('transformer')
        rospy.loginfo("Starting panneau node...")

        self.pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.last_scan = None

        # we sub to /estimation to know when the robot has reached the panel
        rospy.Subscriber('/estimation', PoseWithCovarianceStamped, self.callback_estimation)
        
        period = 0.1
        rospy.Timer(rospy.Duration(period), self.callback_timer)

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
        if self.panel_pose is None:
            return
        
        x = estimation_msg.pose.pose.position.x
        y = estimation_msg.pose.pose.position.y

        if np.sqrt((x - self.panel_pose[0])**2 + (y - self.panel_pose[1])**2) < 0.1:
            self.panel_reached = True


    def publish_goal(self, x, y, theta):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation = Quaternion(*q)
        goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        goal_publisher.publish(goal)


    def go_to_panel(self):
        if self.last_scan is None:
            return
        
        coords = []
        intensities = []
        
        # conversion des points en coordonnées (x,y) et suppression des points infini et des points de faible intensité 
        for i, theta in enumerate(np.arange(self.last_scan.angle_min, self.last_scan.angle_max, self.last_scan.angle_increment)):
            
            if not(self.last_scan.ranges[i] == float("inf")):
                if self.last_scan.ranges[i] > 0.1 and self.last_scan.intensities[i] > 1000:
                    coords.append((self.last_scan.ranges[i]*np.cos(theta), self.last_scan.ranges[i]*np.sin(theta))) 
                    intensities.append(self.last_scan.intensities[i])
        
        # Create a PointCloud2 (si on souhaite les afficher dans rziv, on peut se servir de pc2)
        pc2 = create_cloud(self.last_scan.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])

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

        self.publish_goal(point_a_atteindre[0], point_a_atteindre[1], 0)
        self.state = "TURNING_ROBOT"
        self.panel_pose = point_a_atteindre

    def turn_robot(self):
        # re-calcul pour avoir une bonne orientation face au panneau 
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.panel_pose[0]
        goal_pose.pose.position.y = self.panel_pose[1]

        # orientation du robot
        # TODO, et coder un turn robot dans le controller ?
        
        goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        goal_publisher.publish(goal_pose)

    def callback_timer(self, _):
        print(self.state)
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
