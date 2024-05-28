#!/usr/bin/env python3

from utils import dist_point_to_segment_signed
from pid import PID

from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped, Twist, Quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

import rospy
import numpy as np
from math import atan2, pi
from time import time


MAX_ANG_VEL = 1.

class Controller:
    def __init__(self):
        rospy.init_node("control_node", log_level=rospy.INFO)

        rospy.loginfo("Starting control node...")
       
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.estimate_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.receive_estimate_pose)
        self.path_subscriber = rospy.Subscriber("/path", Path, self.receive_path)
        self.path = None
        self.is_goal_reached_publisher = rospy.Publisher("/is_goal_reached", Empty, queue_size=1)
        

        # pub the goal
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # target position
        self.target_pose = np.array([0., 0., 0.]) #x,y,theta
        # current estimated position
        self.current_pose = np.array([0., 0., 0.]) #x,y,theta

        # control parameters
        self.max_linear_velocity = 0.15
        self.max_angular_velocity = 0.2
        self.max_angular_velocity_while_moving = 0.4

        self.max_accel_lin = 0.05
        self.max_accel_ang = 0.1
        
        self.angle_control_pid = PID(1.0, 0.0, 0.0)
        self.speed_control_pid = PID(1.0, 0.0, 0.0)
        self.dir_correction_pid = PID(5., 0.0, 1.0)

        # State machine : When a goal is received
        # - first we turn to the right direction
        # - then we move to the goal, still adjusting the direction
        self.is_turning = False
        self.is_moving = False

        self.last_time = time()
        self.linear_speed = 0.
        self.angular_speed = 0.


    def receive_estimate_pose(self, msg):
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2])

    def publish_goal(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "odom"

        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0

        q = quaternion_from_euler(0, 0, 0)
        goal_pose.pose.orientation = Quaternion(*q)

        self.goal_publisher.publish(goal_pose)

    def get_next_goal_on_path(self, path_msg):
        """
        Returns the next goal on the path
        Find the nearest line on the path to the robot and returns the end of the line
        """
        # convert the path to a list of points
        path = []
        for pose in path_msg.poses:
            path.append(np.array([pose.pose.position.x, pose.pose.position.y]))

        segs = []
        for i in range(1, len(path)):
            segs.append((path[i-1], path[i]))

        min_dist = float('inf')
        nearest_seg = None
        for seg in segs:
            dist = abs(dist_point_to_segment_signed(self.current_pose[:2], seg[0], seg[1]))
            if dist < min_dist:
                min_dist = dist
                nearest_seg = seg

        return nearest_seg

    def receive_path(self, path_msg):
        self.path = path_msg
        self.target_pose = self.get_next_goal_on_path(self.path)[1] # end of the current segment

        # # if received same goal, return
        # if self.target_pose is not None and almost_equal(new_target_pose, self.target_pose):
        #     return
        # self.target_pose = new_target_pose # TODO USEFUL??

        self.is_turning = True
        self.is_moving = False

        # modulo of the angle
        # if self.target_pose[2] > pi:
        #     self.target_pose[2] -= 2 * pi
        # if self.target_pose[2] < -pi:
        #     self.target_pose[2] += 2 * pi # TODO USEFUL ??

    

    def control_loop(self,_):
        if self.path is not None:
            self.current_segment = self.get_next_goal_on_path(self.path)
            self.target_pose = self.current_segment[1] # end of the current segment

        if self.target_pose is None:  # TODO useful?
            return

        linear_speed, angular_speed = 0., 0.
        error_vector = self.target_pose[:2] - self.current_pose[:2]

        if self.is_turning:
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
                # if we also move, we increase the max_angular_velocity
                if self.is_moving:
                    self.max_angular_velocity = self.max_angular_velocity_while_moving
                else:
                    self.max_angular_velocity = self.max_angular_velocity

                angular_speed = self.angle_control_pid.update(angle_error_rad, period)
                # constraint the angular speed between -MAX_ANG_VEL and MAX_ANG_VEL
                angular_speed = min(self.max_angular_velocity, angular_speed)
                angular_speed = max(-self.max_angular_velocity, angular_speed)
                # print("TURNING: error=", angle_error_rad, "vec_target=", error_vector, "current_pose=", self.current_pose)

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

                # adjust the direction to follow the line
                # compute the dist to the current seg
                dist = 0
                # if the points are different
                if not np.array_equal(self.current_pose[:2], self.current_segment[1]):
                    dist = dist_point_to_segment_signed(self.current_pose[:2], self.current_segment[0], self.current_segment[1])
                correction = self.dir_correction_pid.update(dist, period)
                correction_max = 0.1
                if correction > correction_max:
                    correction = correction_max
                if correction < -correction_max:
                    correction = -correction_max
                angular_speed -= correction
        
        if self.target_pose is not None:
            self.publish_goal(self.target_pose)


        # acceleration
        dt = time() - self.last_time
        self.last_time = time()

        # self._speed is speed at (t)
        # _speed is speed at (t+dt)
        # accel is equal to (_speed - self._speed) / dt
        accel_lin = (linear_speed - self.linear_speed) / dt
        accel_ang = (angular_speed - self.angular_speed) / dt

        if accel_lin > self.max_accel_lin:
            linear_speed = self.linear_speed + self.max_accel_lin * dt
        if accel_lin < -self.max_accel_lin:
            linear_speed = self.linear_speed - self.max_accel_lin * dt
        if accel_ang > self.max_accel_ang:
            angular_speed = self.angular_speed + self.max_accel_ang * dt
        if accel_ang < -self.max_accel_ang:
            angular_speed = self.angular_speed - self.max_accel_ang * dt

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    
        print(linear_speed, angular_speed,"\t", dt)


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