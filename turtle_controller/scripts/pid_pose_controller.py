#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import math
import time

class Controller:
    def __init__(self):
        rospy.init_node("pose_controller")
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_target = rospy.Subscriber("/target_pose", PoseStamped, self.target_callback)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_marker = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.reached = False

        self.prev_dist_error = 0.0
        self.prev_yaw_error = 0.0
        self.integral_dist = 0.0
        self.integral_yaw = 0.0

        self.kp_lin = 0.8
        self.ki_lin = 0.0
        self.kd_lin = 0.1

        self.kp_ang = 1.5
        self.ki_ang = 0.0
        self.kd_ang = 0.2

        self.obstacle_detected = False

        self.trajectory_points = []  # robot path history
        self.circular_waypoints = []
        self.circle_index = 0
        self.in_circular_motion = False

        rospy.set_param('/use_sim_time', True)
        rospy.sleep(3.0)
        rospy.spin()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def target_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        (_, _, self.target_yaw) = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w])

        self.reached = False
        self.in_circular_motion = False
        self.publish_marker(self.target_x, self.target_y, id=1, r=0.0, g=1.0, b=0.0)
        rospy.loginfo(f"[Target] x={self.target_x}, y={self.target_y}, yaw={self.target_yaw:.2f}")

    def scan_callback(self, msg):
        ranges = msg.ranges
        mid = len(ranges) // 2
        front_window = ranges[mid - 10 : mid + 10]
        front_distances = [r for r in front_window if not math.isnan(r)]
        self.obstacle_detected = (front_distances and min(front_distances) < 0.4)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.x = pos.x
        self.y = pos.y
        (_, _, self.yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        # Save trajectory
        self.trajectory_points.append(Point(self.x, self.y, 0.0))
        self.publish_trajectory_marker()
        self.control()

    def control(self):
        cmd = Twist()

        if self.target_x is None or self.target_y is None:
            rospy.loginfo_throttle(5, "Waiting for /target_pose...")
            self.pub_cmd.publish(cmd)
            return

        if self.obstacle_detected:
            rospy.logwarn("Obstacle detected! Rotating...")
            cmd.angular.z = 0.3
            self.pub_cmd.publish(cmd)
            return

        if not self.reached:
            # Navigate to initial target
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            distance = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(angle_to_target - self.yaw)

            if distance > 0.2:
                if abs(yaw_error) > 0.2:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.kp_ang * yaw_error
                else:
                    d_error_dist = distance - self.prev_dist_error
                    cmd.linear.x = self.kp_lin * distance + self.kd_lin * d_error_dist
                    self.prev_dist_error = distance

                    d_error_yaw = yaw_error - self.prev_yaw_error
                    cmd.angular.z = self.kp_ang * yaw_error + self.kd_ang * d_error_yaw
                    self.prev_yaw_error = yaw_error
            else:
                self.reached = True
                self.publish_marker(self.x, self.y, id=2, r=0.0, g=0.0, b=1.0)

                # Offset center behind robot
                cx = self.x - 0.5 * math.sin(self.yaw)
                cy = self.y + 0.5 * math.cos(self.yaw)

                self.generate_circular_waypoints(cx, cy, radius=0.5)
                self.circle_index = 0
                self.in_circular_motion = True
                rospy.loginfo("Target reached. Starting circular trajectory...")

        elif self.in_circular_motion and self.circle_index < len(self.circular_waypoints):
            tx, ty = self.circular_waypoints[self.circle_index]
            dx = tx - self.x
            dy = ty - self.y
            distance = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(angle_to_target - self.yaw)

            cmd.linear.x = 0.15
            cmd.angular.z = 1.2 * yaw_error

            if distance < 0.1:
                self.circle_index += 1
        else:
            cmd = Twist()
            rospy.loginfo_once("Circular trajectory completed.")

        self.pub_cmd.publish(cmd)

    def generate_circular_waypoints(self, center_x, center_y, radius=0.5, steps=36):
        self.circular_waypoints = []
        for angle in range(0, 360, int(360 / steps)):
            rad = math.radians(angle)
            px = center_x + radius * math.cos(rad)
            py = center_y + radius * math.sin(rad)
            self.circular_waypoints.append((px, py))

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.id = 3
        marker.points = [Point(x, y, 0.0) for x, y in self.circular_waypoints]
        self.pub_marker.publish(marker)

    def publish_marker(self, x, y, id=1, r=1.0, g=0.0, b=0.0):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.id = id
        self.pub_marker.publish(marker)

    def publish_trajectory_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_trajectory"
        marker.id = 4
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.8
        marker.color.g = 0.4
        marker.color.b = 0.1
        marker.pose.orientation.w = 1.0
        marker.points = self.trajectory_points
        self.pub_marker.publish(marker)

if __name__ == '__main__':
    Controller()
