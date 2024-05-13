import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from math import cos, sin
import time


class StateEstimation:
    def __init__(self):
        rospy.init_node("state_estimation", log_level=rospy.INFO)

        rospy.loginfo("Démarrage du nœud d'estimation d'état")
       
        self.estimate_publisher = rospy.Publisher("estimation", PoseWithCovarianceStamped, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)

        # Etat (x,y,theta,v,w)
        self.X = np.zeros((5,1))
        self.P = np.zeros((5,5))

        self.last_time_input = rospy.Time.now()
        self.last_time_odom = rospy.Time.now()

        self.v, self.w = 0,0

        # Position de départ
        self.move_position = [0, 0]
        # Position cible à atteindre (X,Y)
        self.target_position = np.array([1.0, 0.0])

        # Paramètres de contrôle
        self.max_linear_velocity = 0.22
        self.max_angular_velocity = 2.84*0.75

    def estimate_state(self,_):
        # X_k|k+1 = f(X_k|k)
        self.X = np.array([
            [float(self.X[0] + period * self.v * cos(self.X[2]))],
            [float(self.X[1] + period * self.v * sin(self.X[2]))],
            [float(self.X[2] + period * self.w)],
            [float(self.v)],
            [float(self.w)]
        ])

        # jacobienne
        F = np.array([
            [1, 0, 1, period*cos(self.X[2]), 0],
            [0, 1, 1, period*sin(self.X[2]), 0],
            [0, 0, 1, 0, period],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # covariance du bruit de modele
        Q = np.array([    
            [0.001, 0, 0, 0, 0],
            [0, 0.001, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0]
        ])

        # P_k|k+1 = F_k*P_k|k*F_k.T + Q
        self.P = np.dot(F, np.dot(self.P,F.T)) + Q

    def floor(self, value, precision):
        return math.floor(value / precision) * precision

    def publish_cmd_vel(self):
        # Publication de la commande de vitesse sur /cmd_vel
        twist_msg = Twist()
        #z = self.floor(self.max_angular_velocity * self.move_position[1], 0.01)
        z = min(self.max_angular_velocity, self.move_position[1])
        x = min(self.max_linear_velocity, self.move_position[0])
        print(f"X : {x} ; Z : {z}")
        twist_msg.linear.x = x
        twist_msg.angular.z = z
        self.cmd_vel_publisher.publish(twist_msg)
       
    def receive_odom(self, odom_msg:Odometry):
        # ÉTAPE DE CORRECTION
        dt = (rospy.Time.now() - self.last_time_odom).to_sec()

        # matrice de correction
        C = np.array([[0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 1]])
       
        # vecteur d'observation Z
        self.v = odom_msg.twist.twist.linear.x # linear velocity
        self.w = odom_msg.twist.twist.angular.z # angular velocity

        Z = np.array([[self.v], [self.w]])

        #covariance de l'observation. matrice de taille 2x2
        Rk = np.array([[0.00001, 0],
                       [0, 0.001]])

        # Kk = Pk|k-1 * C^T * ( C * Pk|k-1 * C^T + Rk )^-1
        self.K = np.dot(self.P, np.dot(C.T, np.linalg.inv(np.dot(np.dot(C, self.P), C.T) + Rk)))
        # Xk|k = Xk-1 + Kk * (Z - C * Xk|k-1)
        self.X = self.X + np.dot(self.K , (Z - np.dot(C, self.X)))
        # Pk|k = ( I - Kk * C ) * Pk|k-1
        self.P = np.dot((np.eye(5) - np.dot(self.K, C)), self.P)

        # Calcul la différence entre la position actuelle et la position cible
        vec_target = self.target_position - self.X[:2, 0]
        self.move_position[0] = np.linalg.norm(vec_target) # vecteur direction

        # Calcul de l'angle à prendre entre la position actuelle et la position cible
        angle_rad = math.atan2(vec_target[1], vec_target[0]) - self.X[2, 0] # angle en radian
        #print(f"vec actuel : {self.X[:2, 0]}, vec target : {vec_target}, angle rad :{angle_rad} ")
        self.move_position[1] = math.sin(abs(angle_rad/2)) # angle / 2 car si angle = pi ou -pi, on veut un déplacement

        self.publish_cmd_vel()

   
    # À adapter
    def publish_estimate(self):
       
        x = self.X[0][0]
        y = self.X[1][0]
        theta = self.X[2][0]

        #print(f"X : {x} ; Y : {y} ; THETA : {theta}")
       
        covariance = np.zeros((3,3)) # Il ne faut que la partie de la covariance concernant x, y et theta
       
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"
       
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
       
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)

        msg.pose.covariance[ 0] = covariance[0][0]
        msg.pose.covariance[ 1] = covariance[0][1]
        msg.pose.covariance[ 5] = covariance[0][2]
        msg.pose.covariance[ 6] = covariance[1][0]
        msg.pose.covariance[ 7] = covariance[1][1]
        msg.pose.covariance[11] = covariance[1][2]
        msg.pose.covariance[30] = covariance[2][0]
        msg.pose.covariance[31] = covariance[2][1]
        msg.pose.covariance[35] = covariance[2][2]
       
        self.estimate_publisher.publish(msg)
   
    def run(self):
        rospy.spin()

       
period = 0.1

if __name__ == "__main__":
    node = StateEstimation()
    #rospy.Subscriber("/odom", Odometry, node.receive_odom)
    rospy.Timer(rospy.Duration(period), node.estimate_state)
    node.run()