#!/usr/bin/env python3

import rospy, queue, os, cv2, pickle

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path

import numpy as np
from math import cos, sin, atan2

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

        # costmap
        self.costmap_size = 800  # points
        self.resolution = 0.05 # 5 cm
        self.costmap = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)
        self.origin_x = 0
        self.origin_y = 0
        
        self.one = False

    def receive_lidar(self, msg):
        # update the costmap
        self.update_costmap(msg)
      
    def receive_estimate_pose(self, msg):
        # Récupération de la position estimée
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2])
        
    def update_costmap(self, scan):
        print(f"costmap updated")
        # update the costmap with the lidar data
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x = self.current_pose[0] + distance * cos(angle + self.current_pose[2])
            y = self.current_pose[1] + distance * sin(angle + self.current_pose[2])
            grid_x = int((y - self.origin_x) / self.resolution + self.costmap_size // 2)
            grid_y = int((x - self.origin_y) / self.resolution + self.costmap_size // 2)

            if 0 <= grid_x < self.costmap_size and 0 <= grid_y < self.costmap_size:
                self.costmap[grid_x, grid_y] = 100  # Mark the cell as occupied


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
        costmap_msg.data = self.costmap.flatten().tolist()
        self.costmap_publisher.publish(costmap_msg)


    def run(self):
        rospy.spin()

    
    def astar(self, start, goal):
        def dist_manhattan(pos1, pos2):
            return np.linalg.norm(np.array(pos2) - np.array(pos1))
            #return abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])
        
        state = Etats(start[0], start[1], 0, [])

        # liste des actions possible pour le robot
        actions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]
        # setup des listes
        liste_chemin = queue.PriorityQueue()
        chemin_deja_visite = set()
        # premire valeur dans la liste
        liste_chemin.put((0, state))
        while not liste_chemin.empty():
            print("iteration")
            # on récupère le tuple en première position dans la liste
            node = liste_chemin.get()
            current_state = node[1]
            print(f"current state pos : {current_state.x, current_state.y}")
            actual_cost = current_state.cost
            # on vérifie si c'est un chemin déjà visité
            if current_state in chemin_deja_visite:
                continue
            chemin_deja_visite.add(current_state)
            for dx, dy in actions:
                next_pos = (current_state.x + dx, current_state.y + dy)
                next_cost = dist_manhattan(next_pos, goal)
                new_state = Etats(current_state.x + dx, current_state.y + dy, next_cost, current_state.liste_actions + [(dx, dy)])
                print(f"new state x : {new_state.x} ; y : {new_state.y} ; costmap : {self.costmap[new_state.y, new_state.x]} ; path cost : {new_state.cost}")
                if self.costmap[next_pos[1], next_pos[0]] == 100:
                    chemin_deja_visite.add(new_state)
                    continue
                # on vérifie si c'est un chemin déjà visité
                if new_state in chemin_deja_visite:
                    continue
                if next_pos == goal:
                    # il faut créer le chemin
                    print(f"nous avons un chemin possible")
                    print(f"liste actions : {new_state.liste_actions}")
                    return new_state.liste_actions
                liste_chemin.put((next_cost, new_state))
        return None
    

    def planning_loop(self, event):
        self.one = True
        start = (self.current_pose[0], self.current_pose[1])
        goal = (self.goal_pose[0], self.goal_pose[1])

        grid_start = (
            int((start[0] - self.origin_x) / self.resolution + self.costmap_size // 2),
            int((start[1] - self.origin_y) / self.resolution + self.costmap_size // 2)
        )
        
        grid_goal = (
            int((goal[0] - self.origin_x) / self.resolution + self.costmap_size // 2),
            int((goal[1] - self.origin_y) / self.resolution + self.costmap_size // 2)
        )
        
        # home_dir = os.path.expanduser("~")
        # save_dir = os.path.join(home_dir, "CM")
        # if not os.path.exists(save_dir):
        #     os.makedirs(save_dir)
        # cv2.imwrite(os.path.join(save_dir, "costmap_image.jpg"), self.costmap)
        
        # # Etudier la costmap pour vérifier le tableau : il est mal positionné ce qui génère un problème dans les emplacements
        # with open(os.path.join(save_dir, 'costmap.pkl'), 'wb') as f:
        #     pickle.dump(self.costmap, f)

        path = self.astar(grid_start, grid_goal)

        # Create a Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"


        poses = []
        current_pos = grid_start

        for action in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            current_pos = (current_pos[0] + action[0], current_pos[1] + action[1])
            pose.pose.position.x = (current_pos[0] - self.costmap_size // 2) * self.resolution + self.origin_x
            pose.pose.position.y = (current_pos[1] - self.costmap_size // 2) * self.resolution + self.origin_y
            pose.pose.position.z = 0

            # Optionally set the orientation of the pose (quaternion)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            poses.append(pose)

        path_msg.poses = poses
        path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(path_msg)

        if path is None:
            return
        
        current_pos = grid_start
        count = 0

        while path and count < 5:
            count += 1
            next_point = path.pop(0)
            current_pos = (current_pos[0] + action[0], current_pos[1] + action[1])

            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "odom"

            goal_pose.pose.position.x = (current_pos[0] - self.costmap_size // 2) * self.resolution + self.origin_x
            goal_pose.pose.position.y = (current_pos[1] - self.costmap_size // 2) * self.resolution + self.origin_y
            goal_pose.pose.position.z = 0

            q = quaternion_from_euler(0, 0, self.goal_pose[2])
            goal_pose.pose.orientation = Quaternion(*q)

            self.goal_publisher.publish(goal_pose)
    

            # TODO : here, launch a A* instance to find the path to the goal from the current position
            # use the costmap to avoid obstacles
            # publish the goal (which will be handled by the controller)
            # pass
        

       
period = 0.2 # seconds

if __name__ == "__main__":
    node = PathPlanner()

    rospy.Timer(rospy.Duration(period), node.planning_loop)
    node.run()