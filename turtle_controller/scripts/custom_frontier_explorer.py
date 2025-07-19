#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatusArray
import tf
import numpy as np
import math
import random
from collections import deque

class FrontierExplorer:
    def __init__(self):
        self.last_map_time = rospy.Time.now()
        self.origin = []
        self.map = []
        self.exploration_done = False

        self.map_size = []
        self.current_pose = []
        self.goal = None
        self.resolution = None
        self.goal_reached = True
        self.rectangle = 0.24
        self.abort_count = 0


        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.frontier_pub = rospy.Publisher('/frontier', PoseArray, queue_size=10)
        self.cluster_pub = rospy.Publisher('/cluster', MarkerArray, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        rospy.Subscriber('/map', OccupancyGrid, self.get_gridmap)
        
        

    def status_callback(self, msg):
        """Handle goal status updates."""
        if not msg.status_list or self.exploration_done:
            self.cmd_pub.publish(Twist())  # Ensure full stop
            return
        latest_status = msg.status_list[-1].status

        # 3 = SUCCEEDED, 4 = ABORTED
        if latest_status == 3:
            rospy.loginfo(" Goal reached. Proceeding to next frontier.")
            self.goal_reached = True
            self.abort_count = 0  # reset on success

        elif latest_status == 4:
            rospy.logwarn(" Goal aborted. Attempting next frontier.")
            self.goal_reached = True
            self.abort_count += 1

            if self.abort_count >= 3:
                if not self.exploration_done:
                    rospy.logwarn("Too many aborts. Executing recovery: Move back and rotate.")
                    self.move_backward(duration=1.5, speed=-0.12)
                    self.rotate_in_place(angle_deg=120, steps=3)
                    self.abort_count = 0
                else:
                    rospy.loginfo("Exploration done. Skipping recovery.")
        if not msg.status_list:
            self.cmd_pub.publish(Twist())  # Ensure full stop
            return



    def rotate_in_place(self, angle_deg=120, steps=3):
        """Rotate the robot in place by a specified angle in degrees."""
        if self.exploration_done:
            self.cmd_pub.publish(Twist())  # stop immediately
            return

        rospy.logwarn("Recovery: Rotating in 3 steps...")

        for i in range(steps):
            rospy.loginfo(f"Rotating step {i+1}/{steps} by {angle_deg} degrees")
            twist = Twist()
            twist.angular.z = 0.4  # rad/s (positive = counter-clockwise)

            # Time required to rotate by angle_deg at angular.z speed
            duration = angle_deg / (abs(twist.angular.z) * 180 / math.pi)

            rate = rospy.Rate(10)
            ticks = int(duration * 10)
            for _ in range(ticks):
                self.cmd_pub.publish(twist)
                rate.sleep()

            self.cmd_pub.publish(Twist())  # stop after each turn
            rospy.sleep(0.5)  # small pause between steps





    def get_odom(self, odom):
        """Extract current pose from odometry message."""
        _, _, yaw = tf.transformations.euler_from_quaternion([
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

    def get_gridmap(self, gridmap):
        """Process the occupancy grid map and publish frontiers."""
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:
            self.last_map_time = gridmap.header.stamp
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            self.origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.resolution = gridmap.info.resolution
            self.map = env
            self.map_size = self.map.shape
            frontiers = self.bfs_frontier(self.map)
            if self.exploration_done:
                return
            if self.goal_reached:
                self.frontier_publisher(frontiers)
                clusters = self.bfs_frontier_segment(frontiers)
                if not clusters:
                    rospy.loginfo("No clusters left. Exploration complete.")

                    # Clear frontiers
                    self.frontier_pub.publish(PoseArray())

                    # Clear clusters
                    delete_all = Marker()
                    delete_all.header.frame_id = "map"
                    delete_all.action = Marker.DELETEALL
                    delete_array = MarkerArray()
                    delete_array.markers.append(delete_all)
                    self.cluster_pub.publish(delete_array)

                    # Stop robot motion
                    self.cmd_pub.publish(Twist())
                    
                    return  # Exit exploration

                marker_array = MarkerArray()
                for idx, cluster in enumerate(clusters):
                    marker_array.markers.append(self.Cluster_publisher(cluster, idx, (random.random(), random.random(), random.random())))
                self.cluster_pub.publish(marker_array)
                goal = self.explore(clusters)
                if goal:
                    self.goal_pubisher(goal)

    def explore(self, clusters):
        """Select a target from the clusters of frontiers."""
        if not clusters:
            self._stop_and_clear()
            return None

        any_valid = False
        while clusters:
            lengths = np.array([len(c) for c in clusters])
            dists = np.array([min([self.distance(p, self.current_pose) for p in c]) for c in clusters])
            scores = lengths / (dists * 10 + 1e-5)
            best_idx = np.argmax(scores)
            cluster = clusters[best_idx]

            dists_to_cluster = np.array([self.distance(p, self.current_pose) for p in cluster])
            avg_dist = np.mean(dists_to_cluster)
            target_idx = np.argmin(abs(dists_to_cluster - avg_dist))
            target = self.map_to_position(cluster[target_idx])

            if self.is_valid(target):
                any_valid = True
                return target
            else:
                del cluster[target_idx]
                if not cluster:
                    del clusters[best_idx]

        
        if not any_valid:
            self._stop_and_clear()
        return None


    def _stop_and_clear(self):
        """Stop the robot and clear all frontiers and clusters."""
        rospy.loginfo("Exploration complete. No more valid frontiers.")
        self.exploration_done = True

        # Stop the robot
        self.cmd_pub.publish(Twist())

        # Clear frontiers
        empty_pose_array = PoseArray()
        empty_pose_array.header.frame_id = "map"
        self.frontier_pub.publish(empty_pose_array)

        # Clear cluster markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header.frame_id = "map"
        self.cluster_pub.publish(MarkerArray(markers=[delete_all]))

        

    def goal_pubisher(self, point_to_go):
        """Publishes the goal with orientation pointing toward the goal from current pose."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()

        goal_pose.pose.position.x = point_to_go[0]
        goal_pose.pose.position.y = point_to_go[1]
        goal_pose.pose.position.z = 0.0

        # Compute direction vector to goal
        dx = point_to_go[0] - self.current_pose[0]
        dy = point_to_go[1] - self.current_pose[1]

        yaw = math.atan2(dy, dx)  # direction from current pose to goal
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]

        self.goal_pub.publish(goal_pose)
        rospy.loginfo(f"Published navigation goal at ({point_to_go[0]}, {point_to_go[1]}) with computed yaw={yaw:.2f}")
        self.goal_reached = False

    def frontier_publisher(self, points):
        """Publish valid frontier points as a PoseArray."""
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        valid_frontiers = []

        for pt in points:
            world_x, world_y = self.map_to_position(pt)
            if self.is_valid((world_x, world_y)):
                pose = Pose()
                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = 0
                pose.orientation.w = 1.0
                valid_frontiers.append(pose)

        
        if len(valid_frontiers) <= 4 or len(valid_frontiers) <= 1:
             # This will explicitly publish a cleared frontier marker
            cleared = PoseArray()
            cleared.header.frame_id = "map"
            cleared.header.stamp = rospy.Time.now()
            self.frontier_pub.publish(cleared)
            return
        pose_array.poses = valid_frontiers
        rospy.loginfo(f"Publishing {len(valid_frontiers)} valid frontiers")
        self.frontier_pub.publish(pose_array)

    def Cluster_publisher(self, points, marker_id, color):
        """Publish a cluster of points as a line strip marker."""
        delete_all = Marker()
        delete_all.header.frame_id = "map"
        delete_all.action = Marker.DELETEALL

        delete_array = MarkerArray()
        delete_array.markers.append(delete_all)

        self.cluster_pub.publish(delete_array)


        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clusters"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for p in points:
            pos = self.map_to_position(p)
            marker.points.append(Point(pos[0], pos[1], 0))
        marker.points.append(marker.points[0])
        return marker

    def bfs_frontier(self, grid):
        """Find frontiers in the occupancy grid."""
        frontiers = []
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i, j] == 0:
                    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
                        ni, nj = i + dx, j + dy
                        if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1] and grid[ni, nj] == -1:
                            frontiers.append((i, j))
                            break
        return frontiers

    def bfs_frontier_segment(self, points):
        """Segment frontiers into clusters using BFS."""
        clusters = []
        visited = set()
        for p in points:
            if p not in visited:
                queue = deque([p])
                visited.add(p)
                segment = []
                while queue:
                    r, c = queue.popleft()
                    segment.append((r, c))
                    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
                        nr, nc = r + dx, c + dy
                        if (nr, nc) in points and (nr, nc) not in visited:
                            visited.add((nr, nc))
                            queue.append((nr, nc))
                if len(segment) >= 20:   
                    clusters.append(segment)
                else:
                    rospy.logwarn(f"Discarding small segment of size {len(segment)}: {segment}")
        return clusters

    def map_to_position(self, coord):
        """Convert grid coordinates to world position."""
        x = coord[0] * self.resolution + self.origin[0]
        y = coord[1] * self.resolution + self.origin[1]
        return [x, y]

    def __position_to_map__(self, p):
        """Convert world position to grid coordinates."""
        x = int((p[0] - self.origin[0]) / self.resolution)
        y = int((p[1] - self.origin[1]) / self.resolution)
        if x < 0 or y < 0 or x >= self.map.shape[0] or y >= self.map.shape[1]:
            return []
        return [x, y]
    
    def move_backward(self, duration=1.0, speed=-0.1):
        """Move the robot backward for a specified duration at a given speed."""
        if self.exploration_done:
            self.cmd_pub.publish(Twist())  # stop immediately
            return
        rospy.logwarn("Recovery: Moving backward...")
        twist = Twist()
        twist.linear.x = speed  # negative = backward
        rate = rospy.Rate(10)
        ticks = int(duration * 10)
        for _ in range(ticks):
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())  # stop


    def is_valid(self, pos):
        """Check if a position is valid for exploration."""
        cell = self.__position_to_map__(pos)
        if not cell:
            return False

        # Reject if the center cell is unknown or occupied
        if self.map[cell[0], cell[1]] != 0:
            return False

        # Define a small square neighborhood around the cell
        r = int(self.rectangle / self.resolution)
        total, free = 0, 0
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = cell[0] + dx, cell[1] + dy
                if 0 <= nx < self.map.shape[0] and 0 <= ny < self.map.shape[1]:
                    total += 1
                    if self.map[nx, ny] == 0:
                        free += 1

        # At least 50% of the surrounding area must be free
        return total > 0 and (free / total) >= 0.3


    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points."""
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

if __name__ == '__main__':
    rospy.init_node('custom_frontier_explorer')
    explorer = FrontierExplorer()
    rospy.spin()
