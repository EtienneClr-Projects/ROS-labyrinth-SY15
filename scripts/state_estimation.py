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
       
        self.estimate_publisher = rospy.Publisher("/estimation", PoseWithCovarianceStamped, queue_size=1)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)

        # Etat (x,y,theta,v,w)
        self.X = np.zeros((5,1))
        self.P = np.zeros((5,5))

        self.last_time_input = rospy.Time.now()
        self.last_time_odom = rospy.Time.now()

        # vitesses lineaire et angulaire
        self.linear_speed, self.angular_speed = 0,0

    def estimate_state(self,_):
        # X_k|k+1 = f(X_k|k)
        self.X = np.array([
            [float(self.X[0] + period * self.linear_speed * cos(self.X[2]))],
            [float(self.X[1] + period * self.linear_speed * sin(self.X[2]))],
            [float(self.X[2] + period * self.angular_speed)],
            [float(self.linear_speed)],
            [float(self.angular_speed)]
        ])

        # jacobienne
        F = np.array([
            [1, 0, 1, period*cos(self.X[2]), 0],
            [0, 1, 1, period*sin(self.X[2]), 0],
            [0, 0, 1, 0, period],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # covariance du bruit de modele (x,y,theta,v,w)
        Q = np.array([    
            [0.001, 0, 0, 0, 0],
            [0, 0.001, 0, 0, 0],
            [0, 0, 0.001, 0, 0],
            [0, 0, 0, 0.001, 0],
            [0, 0, 0, 0, 0.001]
        ])

        # P_k|k+1 = F_k*P_k|k*F_k.T + Q
        self.P = np.dot(F, np.dot(self.P,F.T)) + Q

        self.publish_estimate()

    def receive_odom(self, odom_msg:Odometry):
        # ÉTAPE DE CORRECTION
        
        # matrice de correction
        C = np.array([[0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])
       
        # vecteur d'observation Z
        self.linear_speed = odom_msg.twist.twist.linear.x # linear velocity
        self.angular_speed = odom_msg.twist.twist.angular.z # angular velocity

        Z = np.array([[self.linear_speed], [self.angular_speed]])

        #covariance de l'observation. matrice de taille 2x2
        Rk = np.array([[0.005, 0],
                       [0, 0.005]])

        # Kk = Pk|k-1 * C^T * ( C * Pk|k-1 * C^T + Rk )^-1
        self.K = np.dot(self.P, np.dot(C.T, np.linalg.inv(np.dot(np.dot(C, self.P), C.T) + Rk)))
        # Xk|k = Xk-1 + Kk * (Z - C * Xk|k-1)
        self.X = self.X + np.dot(self.K , (Z - np.dot(C, self.X)))
        # Pk|k = ( I - Kk * C ) * Pk|k-1
        self.P = np.dot((np.eye(5) - np.dot(self.K, C)), self.P)


        # # TODO enlever
        # self.X[0] = odom_msg.pose.pose.position.x
        # self.X[1] = odom_msg.pose.pose.position.y
        # # theta euler
        # self.X[2] = math.atan2(odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w) * 2
        # self.linear_speed = odom_msg.twist.twist.linear.x
        # self.angular_speed = odom_msg.twist.twist.angular.z
        # self.X[3] = self.linear_speed
        # self.X[4] = self.angular_speed



   
    def publish_estimate(self):
       
        x = self.X[0][0]
        y = self.X[1][0]
        theta = self.X[2][0]

        # theta est entre -pi et pi
        if theta > math.pi:
            theta -= 2*math.pi
        elif theta < -math.pi:
            theta += 2*math.pi 
        self.X[2][0] = theta       

        print(f"X : {round(x,3)} ;\t Y : {round(y,3)} ;\t THETA : {round(theta,3)}")
       
        covariance = np.zeros((3,3)) # Il ne faut que la partie de la covariance concernant x, y et theta
       
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
       
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
       
        msg.pose.pose.orientation.x = 0.
        msg.pose.pose.orientation.y = 0.
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

    rospy.Timer(rospy.Duration(period), node.estimate_state)
    node.run()