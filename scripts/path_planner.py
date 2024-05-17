import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import numpy as np
from math import cos, sin, atan2


class PathPlanner:
    def __init__(self):
        rospy.init_node("path_planner_node", log_level=rospy.INFO)
        rospy.loginfo("Démarrage du nœud de planning")

        # sub to the current position
        self.estimate_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.receive_estimate_pose)
        self.current_pose = np.array([0., 0., 0.]) #x,y,theta

        # sub to the lidar
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.receive_lidar)

        # pub the goal
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.goal_pose = np.array([0., 0., 0.])

        # pub the costmap
        self.costmap_publisher = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1)

        # costmap
        self.costmap_size = 500  # points
        self.resolution = 0.05 # 5 cm
        self.costmap = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)        
        self.origin_x = 0
        self.origin_y = 0


    def receive_lidar(self, msg):
        # update the costmap
        self.update_costmap(msg)
      
    def receive_estimate_pose(self, msg):
        # Récupération de la position estimée
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2])
        
    def update_costmap(self, scan):
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

    def planning_loop(self, event):
        # TODO : here, launch a A* instance to find the path to the goal from the current position
        # use the costmap to avoid obstacles
        # publish the goal (which will be handled by the controller)
        pass
        

       
period = 0.2 # seconds

if __name__ == "__main__":
    node = PathPlanner()

    rospy.Timer(rospy.Duration(period), node.planning_loop)
    node.run()