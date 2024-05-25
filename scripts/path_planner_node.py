#!/usr/bin/env python3

import rospy, queue, os, cv2, pickle

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from std_msgs.msg import Empty
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path

import numpy as np
from math import cos, sin, atan2, pi, sqrt

class Etats():
    def __init__(self, x, y, cost, liste_actions):
        self.x = x
        self.y = y
        self.cost = cost
        self.liste_actions = liste_actions

    def __eq__(self, other):
        if hash(self) == hash(other):
            return True
        return False
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __lt__(self, other):
        return self.cost < other.cost
    
    def __str__(self):
        return self.liste_actions


class PathPlanner:
    def __init__(self):
        rospy.init_node("path_planner_node", log_level=rospy.INFO)
        rospy.loginfo("Starting planning node...")

        # sub to the current position
        self.estimate_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.receive_estimate_pose)
        self.current_pose = np.array([0., 0., 0.]) #x,y,theta

        # sub to the lidar
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.receive_lidar)

        # pub the goal
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.goal_pose = np.array([4., -1., 0.])

        # pub the costmap
        self.costmap_publisher = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1)

        # debug path
        self.path_pub = rospy.Publisher('/path_debug', Path, queue_size=10)

        self.is_goal_reached_sub = rospy.Subscriber("/is_goal_reached", Empty, self.receive_goal_reached)
        self.goal_reached = True

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

    def astar(self, start, goal):
        # distance euclidienne entre 2 points
        def dist(pos1, pos2):
            return np.linalg.norm(np.array(pos2) - np.array(pos1))
        
        # calcul de la distance par rapport au mur le plus proche
        # on va chercher à s'en éloigner + recherche dans un rayon de 2 cases autours de la position cible
        def distance_to_nearest_wall(pos):
            x, y = pos
            if self.obstacle_layer[y, x] >= 100:
                return 0
            distances = []
            for i in range(max(0, x-2), min(self.costmap_size, x+3)):
                for j in range(max(0, y-2), min(self.costmap_size, y+3)):
                    if self.obstacle_layer[j, i] >= 100:
                        distances.append(dist((x, y), (i, j)))
            return min(distances) if distances else float('inf')

        grid_start = (
            int((start[0] - self.origin_x) / self.resolution + self.costmap_size // 2),
            int((start[1] - self.origin_y) / self.resolution + self.costmap_size // 2)
        )

        grid_goal = (
            int((goal[0] - self.origin_x) / self.resolution + self.costmap_size // 2),
            int((goal[1] - self.origin_y) / self.resolution + self.costmap_size // 2)
        )

        state = Etats(grid_start[0], grid_start[1], 0, [])

        # liste des actions possible pour le robot
        actions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        # actions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)] # avec diagonales
        # setup des listes
        liste_chemin = queue.PriorityQueue()
        chemin_deja_visite = set()
        # première valeur dans la liste
        liste_chemin.put((0, state))
        while not liste_chemin.empty():
            # on récupère le tuple en première position dans la liste
            node = liste_chemin.get()
            current_state = node[1]
            # on vérifie si c'est un chemin déjà visité
            if current_state in chemin_deja_visite:
                continue
            chemin_deja_visite.add(current_state)
            for dx, dy in actions:
                next_pos = (current_state.x + dx, current_state.y + dy)
                # on vérifie si la prochaine position est dans la grille / costmap : sécurité
                if not (0 <= next_pos[0] < self.costmap_size and 0 <= next_pos[1] < self.costmap_size):
                    continue

                # on vérifie si la position cible est un mur
                if self.obstacle_layer[next_pos[1], next_pos[0]] >= 100:
                    chemin_deja_visite.add(new_state)
                    continue

                # calcul du prochain coût pour la prochaine position
                next_cost = current_state.cost + (sqrt(2) if (dx, dy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)] else 1)
                margin = 3  # marge de sécurité en nombre de cellules
                distance_wall = distance_to_nearest_wall(next_pos)

                # si on est trop proche d'un mur, la position ciblée prend un coup supplémentaire
                if distance_wall < margin:
                    next_cost += (margin - distance_wall) * 10

                # on va chercher à éviter les changements de direction trop souvent
                if current_state.liste_actions:
                    last_action = current_state.liste_actions[-1]
                    # on punie les cas où on doit effectuer un changement de direction par rapport à l'action précédente
                    # tourner, même en diagonal, a un coût
                    if (last_action[0], last_action[1]) != (dx, dy):
                        next_cost += 2

                new_state = Etats(next_pos[0], next_pos[1], next_cost, current_state.liste_actions + [(dx, dy)])

                # on vérifie si c'est un chemin déjà visité
                if new_state in chemin_deja_visite:
                    continue

                # on vérifie si c'est notre destination
                if next_pos == grid_goal:
                    return new_state.liste_actions
                
                # si aucune des conditions précédentes n'est remplie, on l'ajoute à notre liste des chemins à "étudier"
                liste_chemin.put((next_cost + dist(next_pos, grid_goal), new_state))
        return None
    
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

    def publish_goal(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "odom"

        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0

        q = quaternion_from_euler(0, 0, self.goal_pose[2])
        goal_pose.pose.orientation = Quaternion(*q)

        self.goal_publisher.publish(goal_pose)

    def convert_sequence_to_poses_path(self, path, start):
        # path is a sequence of (x,y) movement, with x,y = -1, 0 or 1
        # we convert the sequence to have only the keypoints of the movements
        # for instance the sequence (1,0) (1,0) (1,0) (0,1) will be (3,0) (0,1)
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
        def is_too_near(pos1, pos2):
            return np.linalg.norm(np.array(pos1) - np.array(pos2)) < 0.05
        
        start = (self.current_pose[0], self.current_pose[1])
        goal = (self.goal_pose[0], self.goal_pose[1])

        # print("start",start,"goal",goal)

        print("trying")
        movement_sequence = self.astar(start, goal)
        if movement_sequence is None:
            print("movement seq is None")
            return
        self.goal_reached = False
        print("convert seq")
        poses_path = self.convert_sequence_to_poses_path(movement_sequence, start)

        goal = poses_path[1] # path[0] is the current position

        self.publish_path(poses_path)
        self.publish_goal(goal)
        print("published goal")

    
       
period = 0.2 # seconds

if __name__ == "__main__":
    node = PathPlanner()

    rospy.Timer(rospy.Duration(period), node.planning_loop)
    node.run()