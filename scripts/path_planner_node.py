#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from std_msgs.msg import Empty
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path

import numpy as np
import rospy
from math import cos, sin, atan2, pi, sqrt

from astar import astar

class PathPlanner:
    def __init__(self):
        rospy.init_node("path_planner_node", log_level=rospy.INFO)
        rospy.loginfo("Starting planning node...")

        # sub to the current position
        self.estimate_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.receive_estimate_pose)
        self.current_pose = np.array([0., 0., 0.]) #x,y,theta

        # sub to the lidar
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.receive_lidar)

        # pub the costmap
        self.costmap_publisher = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1)

        # path
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.is_goal_reached_sub = rospy.Subscriber("/is_goal_reached", Empty, self.receive_goal_reached)
        self.goal_reached = True
        self.goal_pose = np.array([-4., -1., 0.])

        # costmap
        self.costmap_size = 200  # points
        self.resolution = 0.05 # 5cm
        self.obstacle_layer = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)
        self.origin_x = 0
        self.origin_y = 0

        self.inflation_layer = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)

        # timer callback update costmap
        self.last_scan = None
        rospy.Timer(rospy.Duration(1.), self.update_costmap_callback)

    def receive_goal_reached(self, msg):
        self.goal_reached = True

    def receive_lidar(self, msg):
        self.update_costmap(msg)
      
    def receive_estimate_pose(self, msg):
        # get estimated position
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2])

    def update_costmap(self, scan):
        # update the costmap with the lidar data
        self.last_scan = scan

    def update_costmap_callback(self, _):
        scan = self.last_scan
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x = self.current_pose[0] + distance * cos(angle + self.current_pose[2])
            y = self.current_pose[1] + distance * sin(angle + self.current_pose[2])
            grid_x = int((x - self.origin_x) / self.resolution + self.costmap_size // 2)
            grid_y = int((y - self.origin_y) / self.resolution + self.costmap_size // 2)

            if 0 <= grid_x < self.costmap_size and 0 <= grid_y < self.costmap_size:
                self.obstacle_layer[grid_y, grid_x] = 100  # Mark the cell as occupied


        # update inflation layer
        self.inflation_layer = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)
        for i in range(self.costmap_size):
            for j in range(self.costmap_size):
                if self.obstacle_layer[j, i] == 100:
                    for k in range(-3, 4):
                        for l in range(-3, 4):
                            if 0 <= i + k < self.costmap_size and 0 <= j + l < self.costmap_size:
                                self.inflation_layer[j + l, i + k] += 10
                    self.inflation_layer[j, i] = 100


        # publish the costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "odom"
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.costmap_size
        costmap_msg.info.height = self.costmap_size
        costmap_msg.info.origin.position.x = -self.costmap_size * self.resolution / 2
        costmap_msg.info.origin.position.y = -self.costmap_size * self.resolution / 2
        costmap_msg.info.origin.position.z = 0
        costmap_msg.info.origin.orientation.x = 0
        costmap_msg.info.origin.orientation.y = 0
        costmap_msg.info.origin.orientation.z = 0
        costmap_msg.info.origin.orientation.w = 1
        # costmap_msg.data = self.inflation_layer.flatten().tolist()
        costmap_msg.data = self.obstacle_layer.flatten().tolist()
        self.costmap_publisher.publish(costmap_msg)


    def run(self):
        rospy.spin()

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        poses = []

        if path is None:
            return

        for action in path:
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

        path_msg.poses = poses
        path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(path_msg)

    def convert_sequence_to_poses_path(self, path, start):
        """
        path is a sequence of (x,y) movement, with x,y = -1, 0 or 1
        we convert the sequence to have only the keypoints of the movements
        for instance the sequence (1,0) (1,0) (1,0) (0,1) will be (3,0) (0,1)
        and then we convert the keypoints to global positions relative to start
        """
        path.append((0,0))
        path = np.array(path)
        last_move = np.array([0,0])
        sum_of_last_moves = np.array([0,0])
        new_sequence = []
        for move in path:
            if np.all(move == last_move):
                sum_of_last_moves += move
                last_move = move
            else:
                last_move = move
                new_sequence.append(sum_of_last_moves)
                sum_of_last_moves = move

        # now we convert each movement as a global position
        current = start
        poses_path = []
        for move in new_sequence:
            current = [
                move[0] * self.resolution + self.origin_x + current[0],
                move[1] * self.resolution + self.origin_y + current[1]
            ]
            poses_path.append(current)

        return poses_path
    
    def planning_loop(self, _):
        # def is_too_near(pos1, pos2): # TODO delete ?
        #     return np.linalg.norm(np.array(pos1) - np.array(pos2)) < 0.05
        
        start = (self.current_pose[0], self.current_pose[1])
        goal = (self.goal_pose[0], self.goal_pose[1])


        movement_sequence = astar(start, goal, self.costmap_size, self.resolution, self.origin_x, self.origin_y, self.obstacle_layer)
        if movement_sequence is None:
            print("movement seq is None")
            return
        self.goal_reached = False

        poses_path = self.convert_sequence_to_poses_path(movement_sequence, start)

        self.publish_path(poses_path)
        print("published path")

    
       
period = 0.2 # seconds

if __name__ == "__main__":
    node = PathPlanner()

    rospy.Timer(rospy.Duration(period), node.planning_loop)
    node.run()