#!/usr/bin/env python3


from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped, Twist
from std_msgs.msg import Empty

import rospy
import numpy as np
from math import atan2, pi
from time import time

from pid import PID

MAX_ANG_VEL = 1.

class Controller:
    def __init__(self):
        rospy.init_node("control_node", log_level=rospy.INFO)

        rospy.loginfo("Starting control node...")
       
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.estimate_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.receive_estimate_pose)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.receive_target_pose)
        self.is_goal_reached_publisher = rospy.Publisher("/is_goal_reached", Empty, queue_size=1)
        

        # target position
        self.target_pose = np.array([0., 0., 0.]) #x,y,theta
        # current estimated position
        self.current_pose = np.array([0., 0., 0.]) #x,y,theta

        # control parameters
        self.max_linear_velocity = 0.15
        self.max_angular_velocity = 0.4
        self.angle_control_pid = PID(1.0, 0.0, 0.0)
        self.speed_control_pid = PID(1.0, 0.0, 0.0)

        # State machine : When a goal is received
        # - first we turn to the right direction
        # - then we move to the goal, still adjusting the direction
        self.is_turning = False
        self.is_moving = False

        self.last_time = time()


    def receive_estimate_pose(self, msg):
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2])

    def receive_target_pose(self, msg):
        def almost_equal(A, B):
            return np.linalg.norm(A-B) < 0.005
                
        new_target_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2
        ])

        # if received same goal, return
        if self.target_pose is not None and almost_equal(new_target_pose, self.target_pose):
            return
        self.target_pose = new_target_pose

        self.is_turning = True
        self.is_moving = False

        # modulo of the angle
        if self.target_pose[2] > pi:
            self.target_pose[2] -= 2 * pi
        if self.target_pose[2] < -pi:
            self.target_pose[2] += 2 * pi

    def control_loop(self,_):
        if self.target_pose is None:
            return

        linear_speed, angular_speed = 0., 0.
        error_vector = self.target_pose - self.current_pose

        if self.is_turning:
            if self.is_moving: # reduce the angular speed when moving
                self.max_angular_velocity = 0.4
            else:
                self.max_angular_velocity = MAX_ANG_VEL
            angle_error_rad = atan2(error_vector[1], error_vector[0]) - self.current_pose[2]
            # modulo
            if angle_error_rad > pi:
                angle_error_rad -= 2 * pi
            if angle_error_rad < -pi:
                angle_error_rad += 2 * pi

            # if we are close enough to the targeted angle, we stop
            if abs(angle_error_rad) < 30*pi/180 and not self.is_moving or self.is_moving and abs(angle_error_rad)< 0.005:
                # print("ANGLE OK")
                # self.is_turning = False # uncomment to do only translation in the second phase
                self.is_moving = True
                angular_speed = 0.
            else:
                angular_speed = self.angle_control_pid.update(angle_error_rad, period)
                # constraint the angular speed between -MAX_ANG_VEL and MAX_ANG_VEL
                angular_speed = min(self.max_angular_velocity, angular_speed)
                angular_speed = max(-self.max_angular_velocity, angular_speed)
                # print("TURNING: error=", angle_rad, "vec_target=", error_vector, "current_pose=", self.current_pose)

        if self.is_moving:
            # if we are close enough to the target, we stop
            if np.linalg.norm(error_vector[:2]) < 0.05:
                # print("POSITION OK", error_vector[:2])
                self.is_moving = False
                self.is_turning = False
                self.target_pose = None
                linear_speed = 0.
                angular_speed = 0.

                self.is_goal_reached_publisher.publish(Empty())
            else:
                distance_error_m = np.linalg.norm(error_vector[:2])
                linear_speed = self.speed_control_pid.update(distance_error_m, period)
                # constraint the linear speed between -MAX_LIN_VEL and MAX_LIN_VEL
                linear_speed = min(self.max_linear_velocity, linear_speed)
                linear_speed = max(-self.max_linear_velocity, linear_speed)
                # print("MOVING: ", linear_speed, "error=", error_vector[:2], "current_pose=", self.current_pose)

        self.publish_cmd_vel(linear_speed,angular_speed)

    def publish_cmd_vel(self,linear_speed,angular_speed):
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