import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from math import cos, sin
import time


class Controller:
    def __init__(self):
        rospy.init_node("control_node", log_level=rospy.INFO)

        rospy.loginfo("Démarrage du nœud de contrôle")
       
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.estimate_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.receive_estimate_pose)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.receive_target_pose)

        # Position cible à atteindre (X,Y)
        self.target_pose = np.array([0., 0., 0.]) #x,y,theta
        # Position actuelle estimée
        self.current_pose = np.array([0., 0., 0.]) #x,y,theta

        # Paramètres de contrôle
        self.max_linear_velocity = 0.3
        self.max_angular_velocity = 0.5
        self.last_speed = 0.

        # Machine a état. D'abord on tourne dans la bonne direction, puis on avance
        self.is_turning = False
        self.is_moving = False

    def receive_estimate_pose(self, msg):
        # Récupération de la position estimée
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     # quat to euler
                                     math.atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z + msg.pose.pose.orientation.x*msg.pose.pose.orientation.y), 1-2*(msg.pose.pose.orientation.y**2 + msg.pose.pose.orientation.z**2))
                                     ])

    def receive_target_pose(self, msg):
        print("#################")
        print(f"Position cible reçue : {msg.pose.position.x}, {msg.pose.position.y}")
        print("#################")
        # Récupération de la position cible
        self.target_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.orientation.z
        ])
        self.is_turning = True
        self.is_moving = False

    def control_loop(self,_):
        linear_speed, angular_speed = 0., 0.
        vec_target = self.target_pose - self.current_pose

        if self.is_turning:
            angle_rad = math.atan2(vec_target[1], vec_target[0]) - self.current_pose[2]

            if abs(angle_rad) < 0.005:
                print("ANGLE OK")
                self.is_turning = False
                self.is_moving = True
            else:
                #print(f"vec actuel : {self.X[:2, 0]}, vec target : {vec_target}, angle rad :{angle_rad} ")
                angular_speed = angle_rad # angle / 2 car si angle = pi ou -pi, on veut un déplacement
                # min max avec max_angular_velocity en fonction de si negatif ou positif
                angular_speed = min(self.max_angular_velocity, angular_speed)
                angular_speed = max(-self.max_angular_velocity, angular_speed)
                print("TURNING: ", angular_speed, "error=", angle_rad, "vec_target=", vec_target, "current_pose=", self.current_pose)

        if self.is_moving:
            # Calcul la différence entre la position cible et la position actuelle
            if np.linalg.norm(vec_target[:2]) < 0.01:
                print("POSITION OK", vec_target[:2])
                self.is_moving = False
            else:
                linear_speed = vec_target[0] * cos(self.current_pose[2]) + vec_target[1] * sin(self.current_pose[2])
                linear_speed = min(self.max_linear_velocity, linear_speed)
                linear_speed = max(-self.max_linear_velocity, linear_speed)

                # max accel
                accel = 0.05
                if abs(linear_speed - self.last_speed) > accel:
                    linear_speed = self.last_speed + accel if linear_speed > self.last_speed else self.last_speed - accel

                self.last_speed = linear_speed
                print("MOVING: ", linear_speed, "error=", vec_target[:2], "current_pose=", self.current_pose)
        print("linear_speed", linear_speed, "angular_speed", angular_speed)
        self.publish_cmd_vel(linear_speed,angular_speed)

    def publish_cmd_vel(self,linear_speed,angular_speed):
        # Publication de la commande de vitesse sur /cmd_vel
        twist_msg = Twist()
        # print(f"lin : {round(linear_speed,2)} \t ang : {round(angular_speed,2)} \n error: {self.target_pose - self.current_pose}")
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_msg)
       
    def run(self):
        rospy.spin()

       
period = 0.1

if __name__ == "__main__":
    node = Controller()

    rospy.Timer(rospy.Duration(period), node.control_loop)
    node.run()