#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from math import cos, sin
import numpy as np
import math


class StateEstimation:
    def __init__(self):
        rospy.init_node("state_estimation", log_level=rospy.INFO)

        rospy.loginfo("Starting estimate state node...")
       
        self.estimate_publisher = rospy.Publisher("/estimation", PoseWithCovarianceStamped, queue_size=1)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)

        # State (x,y,theta,v,w)
        self.X = np.zeros((5,1))
        self.P = np.eye(5) * 0.00001
        # self.P = np.zeros((5,5))

        self.last_time_input = rospy.Time.now()
        self.last_time_odom = rospy.Time.now()

        # linear and angular speeds
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

        # jacobian
        F = np.array([
            [1, 0, 1, period*cos(self.X[2]), 0],
            [0, 1, 1, period*sin(self.X[2]), 0],
            [0, 0, 1, 0, period],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # model noise covariance (x,y,theta,v,w)
        Q = np.array([    
            [0.000001, 0, 0, 0, 0],
            [0, 0.00001, 0, 0, 0],
            [0, 0, 0.000001, 0, 0],
            [0, 0, 0, 0.00000001, 0],
            [0, 0, 0, 0, 0.00000001]
        ])

        #Q = np.array([[3.818390720793937, -0.0038690045079549226, -0.0036724385876110506, 0, 0],[-0.0038690045079549226, 0.0034582841925877537, -0.0011010664538815115, 0, 0],[-0.0036724385876110506, -0.0011010664538815115, 0.0864601243579371, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])
        #Q = np.array([[1.2702830925661237, 0, 0, 0, 0],[0, -0.0005039289230828935, 0, 0, 0],[0, 0, 0.027228873105481513, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])

        # Q = np.array([    
        #     [0.00000001, 0, 0, 0, 0],
        #     [0, 0.00000001, 0, 0, 0],
        #     [0, 0, 0.00000001, 0, 0],
        #     [0, 0, 0, 0.00000001, 0],
        #     [0, 0, 0, 0, 0.00000001]
        # ])

        #Q = np.array([[3.412708566019642, 0, 0, 0, 0],[0, 0.012739002621238978, 0, 0, 0],[0, 0, 0.035042013292932966, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])
        #Q = np.array([[3.2630235512462895, 0, 0, 0, 0],[0, -0.04399235972803118, 0, 0, 0],[0, 0, 0.04590247297961749, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])
        #Q = np.array([[1.8669314621738338, 0, 0, 0, 0],[0, 0.02698389423956492, 0, 0, 0],[0, 0, 0.02925634322075697, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])
        #Q = np.array([[0.10239125972415465, 0, 0, 0, 0],[0, 0.0031169608481930212, 0, 0, 0],[0, 0, 0.0063528874860538055, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])
        #Q = np.array([[-0.0001432835040265341, 0, 0, 0, 0],[0, 0.0007788287943420276, 0, 0, 0],[0, 0, 0.02119994116152982, 0, 0],[0, 0, 0, 0.00000001, 0],[0, 0, 0, 0, 0.00000001]])

        # P_k|k+1 = F_k*P_k|k*F_k.T + Q
        self.P = np.dot(F, np.dot(self.P,F.T)) + Q

        self.publish_estimate()

    def receive_odom(self, odom_msg:Odometry):
       
        # correction matrix
        C = np.array([[0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])
       
        # observation vector Z
        self.linear_speed = odom_msg.twist.twist.linear.x # linear velocity
        self.angular_speed = odom_msg.twist.twist.angular.z # angular velocity

        # x, y, theta = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 2*math.atan2(2*odom_msg.pose.pose.orientation.z, 2*odom_msg.pose.pose.orientation.w)
        # self.X = np.array([[x], [y], [theta], [self.linear_speed], [self.angular_speed]])

        Z = np.array([[self.linear_speed], [self.angular_speed]])

        # observation covariance matrix 2x2
        Rk = np.array([[0.000000001, 0],
                       [0, 0.000000001]])
        
        #Rk = np.array([[0.001287785929683632, 0.001254101140910568],[0.001254101140910568, 0.005586995789849172]])
        
        #Rk = np.array([[22.657998307270393, 0],[0, 22.752765685887965]])
        #Rk = np.array([[24.63476751628528, 0],[0, 24.24973876296307]])
        #Rk = np.array([[20, 0],[0, 20]])

        # Kk = Pk|k-1 * C^T * ( C * Pk|k-1 * C^T + Rk )^-1
        # Xk|k = Xk-1 + Kk * (Z - C * Xk|k-1)
        # Pk|k = ( I - Kk * C ) * Pk|k-1
        self.K = np.dot(self.P, np.dot(C.T, np.linalg.inv(np.dot(np.dot(C, self.P), C.T) + Rk)))
        self.X = self.X + np.dot(self.K , (Z - np.dot(C, self.X)))
        self.P = np.dot((np.eye(5) - np.dot(self.K, C)), self.P)

   
    def publish_estimate(self):
       
        x = self.X[0][0]
        y = self.X[1][0]
        theta = self.X[2][0]

        # theta between -pi and pi
        if theta > math.pi:
            theta -= 2*math.pi
        elif theta < -math.pi:
            theta += 2*math.pi 
        self.X[2][0] = theta       

        # print(f"X : {round(x,3)} ;\t Y : {round(y,3)} ;\t THETA : {round(theta,3)}")
       
        covariance = np.zeros((3,3)) # We only need (x,y,theta)
       
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

       
period = 0.05

if __name__ == "__main__":
    node = StateEstimation()

    rospy.Timer(rospy.Duration(period), node.estimate_state)
    node.run()