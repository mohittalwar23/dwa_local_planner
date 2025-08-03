#!/usr/bin/env python3
"""
DWA Local Planner for ROS 2

This node implements a Dynamic Window Approach (DWA) local planner for navigation.
It subscribes to laser scan, odometry, and goal topics, and publishes velocity commands
based on cost-optimized trajectory predictions. Trajectories are visualized in RViz.
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
from typing import List, Tuple
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

class DWAConfig:
    """
    Configuration parameters for the DWA planner.
    """
    def __init__(self):
        self.max_linear_vel = 0.5
        self.min_linear_vel = 0.0
        self.max_angular_vel = 1.5
        self.min_angular_vel = -1.5
        self.max_linear_accel = 2.0
        self.max_angular_accel = 3.0
        self.linear_vel_samples = 10
        self.angular_vel_samples = 30  # finer angular resolution
        self.prediction_time = 2.0     # shorter prediction
        self.dt = 0.1
        self.goal_cost_weight = 0.8
        self.obstacle_cost_weight = 4.0
        self.velocity_cost_weight = 0.3
        self.robot_radius = 0.2
        self.min_obstacle_distance = 0.3
        self.goal_tolerance = 0.2

class DWALocalPlanner(Node):
    """
    DWA Local Planner ROS 2 Node.
    """
    def __init__(self):
        super().__init__('dwa_local_planner')
        self.config = DWAConfig()
        self.current_pose = None
        self.current_twist = None
        self.laser_data = None
        self.goal_pose = None
        self.goal_queue = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos)
        self.rviz_goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.trajectory_vis_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("DWA Local Planner initialized")

    def odom_callback(self, msg):
        """Callback for odometry updates."""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def laser_callback(self, msg):
        """Callback for laser scan data."""
        self.laser_data = msg

    def goal_callback(self, msg):
        """Callback for goal input. Queues multiple waypoints."""
        self.get_logger().info(f"Received goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.goal_queue.append(msg.pose)
        if self.goal_pose is None:
            self.goal_pose = self.goal_queue.pop(0)

    def control_loop(self):
        """
        Main control loop called by ROS 2 timer.
        Selects and executes the best trajectory using DWA.
        """
        if not self.is_ready():
            return

        if self.is_goal_reached():
            self.stop_robot()
            self.get_logger().info("Goal reached!")

            if self.goal_queue:
                self.goal_pose = self.goal_queue.pop(0)
                self.get_logger().info(f"Fetching next goal from queue: x={self.goal_pose.position.x:.2f}, y={self.goal_pose.position.y:.2f}")
            else:
                self.goal_pose = None  # Only clear if no goals left

            return


        best_cmd = self.dwa_planning()
        if best_cmd is not None:
            cmd_msg = Twist()
            cmd_msg.linear.x = best_cmd[0]
            cmd_msg.angular.z = best_cmd[1]
            self.cmd_vel_pub.publish(cmd_msg)
        else:
            self.stop_robot()
            self.get_logger().warn("No safe trajectory found - stopping robot")

    def is_ready(self):
        """Check if all required sensor inputs and goals are available."""
        return all([
            self.current_pose is not None,
            self.current_twist is not None,
            self.goal_pose is not None,
            self.laser_data is not None
        ])

    def is_goal_reached(self):
        """Return True if current pose is within goal tolerance."""
        if self.goal_pose is None or self.current_pose is None:
            return False
        dx = self.current_pose.position.x - self.goal_pose.position.x
        dy = self.current_pose.position.y - self.goal_pose.position.y
        return math.sqrt(dx ** 2 + dy ** 2) < self.config.goal_tolerance

    def stop_robot(self):
        """Publishes a zero velocity command."""
        self.cmd_vel_pub.publish(Twist())

    def dwa_planning(self):
        """
        Main DWA planning routine.
        Evaluates all feasible velocity commands and selects the one with the lowest cost.
        Includes early stopping if no progress is made.
        """
        early_stop_limit = 15  # Number of iterations to wait before declaring "stuck"
        stagnant_counter = 0
        last_best_cost = float('inf')

        velocity_window = self.generate_dynamic_window()
        best_cost = float('inf')
        best_cmd = None
        all_trajectories = []

        if not velocity_window:
            return None

        for linear_vel, angular_vel in velocity_window:
            trajectory = self.predict_trajectory(linear_vel, angular_vel)
            if not trajectory:
                continue

            cost = self.evaluate_trajectory(trajectory, linear_vel, angular_vel)
            all_trajectories.append((trajectory, cost))

            if cost < best_cost:
                best_cost = cost
                best_cmd = (linear_vel, angular_vel)

        # Early stopping logic
        if best_cost == last_best_cost:
            stagnant_counter += 1
        else:
            stagnant_counter = 0
            last_best_cost = best_cost

        if stagnant_counter >= early_stop_limit:
            print("[DWA] Goal not reachable. Waiting for next goal.")
            return None

        self.visualize_trajectories(all_trajectories, best_cmd)
        return best_cmd

    def generate_dynamic_window(self):
        """
        Generates a set of velocity commands based on current velocity
        and the acceleration limits defined in the configuration.
        """
        if self.current_twist is None:
            curr_lv, curr_av = 0.0, 0.0
        else:
            curr_lv = self.current_twist.linear.x
            curr_av = self.current_twist.angular.z

        dlv = self.config.max_linear_accel * self.config.dt
        dav = self.config.max_angular_accel * self.config.dt

        min_lv = max(self.config.min_linear_vel, curr_lv - dlv)
        max_lv = min(self.config.max_linear_vel, curr_lv + dlv)
        min_av = max(self.config.min_angular_vel, curr_av - dav)
        max_av = min(self.config.max_angular_vel, curr_av + dav)

        lv_step = (max_lv - min_lv) / max(1, self.config.linear_vel_samples - 1)
        av_step = (max_av - min_av) / max(1, self.config.angular_vel_samples - 1)

        return [(min_lv + i * lv_step, min_av + j * av_step)
                for i in range(self.config.linear_vel_samples)
                for j in range(self.config.angular_vel_samples)]

    def predict_trajectory(self, lv, av):
        """
        Predicts the robot's trajectory over a short horizon using given velocities.
        """
        traj = []
        x, y, theta = 0.0, 0.0, 0.0
        steps = int(self.config.prediction_time / self.config.dt)
        for _ in range(steps):
            x += lv * math.cos(theta) * self.config.dt
            y += lv * math.sin(theta) * self.config.dt
            theta += av * self.config.dt
            traj.append((x, y))
        return traj

    def evaluate_trajectory(self, traj, lv, av):
        """
        Calculates the cost of a trajectory using a weighted sum of
        goal distance, obstacle proximity, and velocity.
        """
        if not traj:
            return float('inf')
        goal_cost = self.calculate_goal_cost(traj[-1])
        obs_cost = self.calculate_obstacle_cost(traj)
        vel_cost = self.calculate_velocity_cost(lv, av)
        if obs_cost == float('inf'):
            return float('inf')
        return (self.config.goal_cost_weight * goal_cost +
                self.config.obstacle_cost_weight * obs_cost +
                self.config.velocity_cost_weight * vel_cost)

    def calculate_goal_cost(self, endpoint):
        """Returns Euclidean distance from predicted endpoint to goal."""
        if self.goal_pose is None or self.current_pose is None:
            return float('inf')
        cp = self.current_pose.position
        orientation = self.current_pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        ex = cp.x + endpoint[0] * math.cos(yaw) - endpoint[1] * math.sin(yaw)
        ey = cp.y + endpoint[0] * math.sin(yaw) + endpoint[1] * math.cos(yaw)
        dx = self.goal_pose.position.x - ex
        dy = self.goal_pose.position.y - ey
        return math.sqrt(dx ** 2 + dy ** 2)

    def calculate_obstacle_cost(self, traj):
        """
        Checks if the trajectory is safe with respect to LaserScan data.
        Returns inverse distance to closest obstacle.
        """
        if self.laser_data is None:
            return 0.0
        min_dist = float('inf')
        for x, y in traj:
            d = math.sqrt(x ** 2 + y ** 2)
            a = math.atan2(y, x)
            idx = self.angle_to_laser_index(a)
            if 0 <= idx < len(self.laser_data.ranges):
                l_dist = self.laser_data.ranges[idx]
                if l_dist < float('inf') and d + self.config.robot_radius > l_dist:
                    return float('inf')
                if l_dist < float('inf'):
                    buffer = l_dist - d - self.config.robot_radius
                    if buffer >= 0:
                        min_dist = min(min_dist, buffer)
        return 1.0 / max(min_dist, 0.01) if min_dist < self.config.min_obstacle_distance else 0.0

    def angle_to_laser_index(self, angle):
        """Converts a given angle to a corresponding index in the LaserScan array."""
        if self.laser_data is None:
            return -1
        angle = angle - self.laser_data.angle_min
        index = int(angle / self.laser_data.angle_increment)
        return max(0, min(index, len(self.laser_data.ranges) - 1))

    def calculate_velocity_cost(self, lv, av):
        """Returns normalized cost for velocity magnitude."""
        return abs(lv) / self.config.max_linear_vel + abs(av) / self.config.max_angular_vel

    def visualize_trajectories(self, trajectories, best_cmd):
        """
        Publishes all sampled trajectories to RViz for visualization.
        Colors reflect cost: green for best, red for collision, blue-magenta gradient for others.
        """
        marker_array = MarkerArray()
        for i, (traj, cost) in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.color.a = 0.5
            marker.scale.x = 0.02
            if best_cmd is not None and i == 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                if cost == float('inf'):
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    norm = min(cost / 10.0, 1.0)
                    marker.color.r = norm
                    marker.color.g = 0.0
                    marker.color.b = 1.0 - norm
            for x, y in traj:
                p = Point()
                p.x, p.y = x, y
                p.z = 0.0
                marker.points.append(p)
            marker_array.markers.append(marker)
        self.trajectory_vis_pub.publish(marker_array)

def main(args=None):
    """ROS 2 node entry point."""
    rclpy.init(args=args)
    planner = DWALocalPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
