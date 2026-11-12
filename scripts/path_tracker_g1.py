#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import copy


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('critical_angle', 60.0 * math.pi / 180.0),
                ('curvature_scaling_angle', 30.0 * math.pi / 180.0),
                ('dt', 0.05),
                ('desired_vel', 0.5),
                ('max_lin_vel', 0.5),
                ('min_lin_vel', 0.0),
                ('max_ang_vel', 0.2),
                ('min_ang_vel', -0.2),
                ('max_lin_acc', 1.8),
                ('max_lin_dec', 1.8),
                ('max_ang_acc', 0.5),
                ('stop_lin_dec', 0.5),  # Gentle goal-stop deceleration, m/s^2.
                ('look_ahead_distance', 0.8),
                ('look_ahead_error_margin', 0.5),
                ('goal_reaching_threshold', 0.5),
                ('fixed_frame', 'map'),
                ('robot_frame', 'mimosa_body'),
            ]
        )

        self.load_params()

        # State variables
        self.path_to_exec = []
        self.current_pose = Pose()
        self.current_vel = Twist()
        self.cmd_vel = Twist()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.carrot_point_pub = self.create_publisher(PoseStamped, 'carrot_point', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry',
            self.odom_callback,
            1
        )

        self.path_sub = self.create_subscription(
            Path,
            'path',
            self.path_callback,
            10
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Pure Pursuit Controller initialized')

    def load_params(self):
        """Load parameters from ROS2 parameter server"""
        self.critical_angle = self.get_parameter('critical_angle').value
        self.curvature_scaling_angle = self.get_parameter('curvature_scaling_angle').value
        self.dt = self.get_parameter('dt').value
        self.desired_vel = self.get_parameter('desired_vel').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.min_lin_vel = self.get_parameter('min_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.min_ang_vel = self.get_parameter('min_ang_vel').value
        self.max_lin_acc = self.get_parameter('max_lin_acc').value
        self.max_lin_dec = self.get_parameter('max_lin_dec').value
        self.max_ang_acc = self.get_parameter('max_ang_acc').value
        self.stop_lin_dec = self.get_parameter('stop_lin_dec').value
        for name in ('max_ang_acc', 'stop_lin_dec'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero')
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.look_ahead_error_margin = self.get_parameter('look_ahead_error_margin').value
        self.goal_reaching_threshold = self.get_parameter('goal_reaching_threshold').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

    def odom_callback(self, msg):
        """Handle odometry messages"""
        self.current_pose = msg.pose.pose
        self.current_vel = msg.twist.twist

    def calculate_distance(self, p1, p2):
        """Calculate 2D Euclidean distance between two poses"""
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return math.sqrt(dx**2 + dy**2)

    def calculate_path_length(self, path):
        """Calculate total path length"""
        dist = 0.0
        for i in range(1, len(path)):
            dist += self.calculate_distance(path[i].pose, path[i-1].pose)
        return dist

    def path_callback(self, msg):
        """Handle new path messages"""
        self.path_to_exec = list(msg.poses)
        self.get_logger().info(f'Received new path with {len(self.path_to_exec)} waypoints')

    def control_loop(self):
        """Main control loop called at fixed rate"""
        # print('Control loop iteration')
        # Check if path is empty
        if len(self.path_to_exec) <= 0:
            # Finish braking and keep refreshing the stopped command.
            self.stop_robot()
            return

        # Check if goal is reached
        # print('last point in path:', self.path_to_exec[-1].pose.position)
        # print('current pose:', self.current_pose.position)
        # print('distance to goal:', self.calculate_distance(self.path_to_exec[-1].pose, self.current_pose), 'goal threshold:', self.goal_reaching_threshold)
        if self.calculate_distance(self.path_to_exec[-1].pose, self.current_pose) < self.goal_reaching_threshold:
            self.get_logger().info('Reached goal tolerance; braking to a stop')
            self.path_to_exec = []
            self.stop_robot()
            return
        # print('calculated distance to goal')

        # Compute and publish velocity command
        if self.compute_command_velocity():
            # Enforce forward-only operation and maximum forward speed.
            self.cmd_vel.linear.x = max(
                0.0, min(self.cmd_vel.linear.x, self.max_lin_vel)
            )
            target_vx = self.cmd_vel.linear.x
            previous_vx = getattr(self, "_last_cmd_vx", 0.0)

            if target_vx == 0.0:
                # Stop immediately when rotation in place is required.
                limited_vx = 0.0
            else:
                # Limit acceleration and deceleration.
                rate = (self.max_lin_acc if target_vx > previous_vx
                        else self.max_lin_dec)
                max_change = rate * self.dt
                limited_vx = previous_vx + max(
                    -max_change,
                    min(max_change, target_vx - previous_vx)
                )

            self.cmd_vel.linear.x = limited_vx
            self._last_cmd_vx = limited_vx

            # Limit angular acceleration to prevent oscillating commands.
            target_wz = max(
                -self.max_ang_vel,
                min(self.max_ang_vel, self.cmd_vel.angular.z)
            )
            previous_wz = getattr(self, "_last_cmd_wz", 0.0)

            max_change = self.max_ang_acc * self.dt

            limited_wz = previous_wz + max(
                -max_change,
                min(max_change, target_wz - previous_wz)
            )

            self.cmd_vel.angular.z = limited_wz
            self._last_cmd_wz = limited_wz

            self.cmd_vel_pub.publish(self.cmd_vel)
        else:
            self.get_logger().warn("Can't track given path")
            self.stop_robot(immediate=True)

    def stop_robot(self, immediate=False):
        """Brake over successive timer ticks; stop immediately on tracking failure."""
        vel = Twist()
        if not immediate:
            previous_vx = getattr(self, '_last_cmd_vx', 0.0)
            previous_wz = getattr(self, '_last_cmd_wz', 0.0)

            # Scale both commands together to preserve the turning radius
            # while braking, avoiding a final turn in place.
            fraction = 1.0
            if previous_vx != 0.0:
                deceleration = min(self.max_lin_dec, self.stop_lin_dec)
                fraction = min(
                    fraction, deceleration * self.dt / abs(previous_vx)
                )
            if previous_wz != 0.0:
                fraction = min(
                    fraction, self.max_ang_acc * self.dt / abs(previous_wz)
                )

            scale = max(0.0, 1.0 - fraction)
            vel.linear.x = previous_vx * scale
            vel.angular.z = previous_wz * scale

        self.cmd_vel = vel
        self._last_cmd_vx = vel.linear.x
        self._last_cmd_wz = vel.angular.z
        self.cmd_vel_pub.publish(self.cmd_vel)

    def compute_command_velocity(self):
        # print('computing velocity commands')
        """Compute velocity commands using pure pursuit algorithm"""
        if len(self.path_to_exec) == 0:
            # print('path empty')
            return False

        # print('0')
        # Find closest point on the path
        dists = [self.calculate_distance(self.current_pose, wp.pose)
                 for wp in self.path_to_exec]
        # print('1')
        closest_waypoint_ind = np.argmin(dists)
        # print('2')

        # Prune path to the closest waypoint
        self.path_to_exec = self.path_to_exec[closest_waypoint_ind:]
        # print('pruned path:')
        # for wp in self.path_to_exec:
        #     print(f'  waypoint: {wp.pose.position.x}, {wp.pose.position.y}, {wp.pose.position.z}')
        # print('3')

        # Find carrot waypoint
        carrot_point_ind = len(self.path_to_exec) - 1
        # print('4')
        for i in range(len(self.path_to_exec)):
            # print('5', i, '0')
            if self.calculate_distance(self.current_pose, self.path_to_exec[i].pose) >= self.look_ahead_distance:
                carrot_point_ind = i
                break
            # print('5', i, '1')

        # Carrot waypoint to follow
        carrot_waypoint = copy.deepcopy(self.path_to_exec[carrot_point_ind].pose)
        # print('6')

        # Interpolate carrot point if it's too far
        if (self.calculate_distance(self.current_pose, self.path_to_exec[carrot_point_ind].pose) >=
            (self.look_ahead_distance + self.look_ahead_error_margin) and carrot_point_ind != 0):

            v2 = np.array([
                self.path_to_exec[carrot_point_ind].pose.position.x,
                self.path_to_exec[carrot_point_ind].pose.position.y,
                self.path_to_exec[carrot_point_ind].pose.position.z
            ])
            v1 = np.array([
                self.path_to_exec[carrot_point_ind-1].pose.position.x,
                self.path_to_exec[carrot_point_ind-1].pose.position.y,
                self.path_to_exec[carrot_point_ind-1].pose.position.z
            ])

            dir_vec = v2 - v1
            dir_norm = np.linalg.norm(dir_vec)

            d = 0.0
            while d < dir_norm:
                v = v1 + (v2 - v1) / dir_norm * d
                cv = np.array([
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.current_pose.position.z
                ])
                if np.linalg.norm(v - cv) >= self.look_ahead_distance:
                    carrot_waypoint.position.x = v[0]
                    carrot_waypoint.position.y = v[1]
                    carrot_waypoint.position.z = v[2]
                    break
                d += self.look_ahead_error_margin

        # print('7')
        # Update path with carrot waypoint
        carrot_waypoint_stamped = PoseStamped()
        # print('8')
        carrot_waypoint_stamped.pose = carrot_waypoint
        # print('9')
        # self.path_to_exec = [carrot_waypoint_stamped] + self.path_to_exec[carrot_point_ind:]
        # print('10')

        # Transform carrot point into robot frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.fixed_frame,
                rclpy.time.Time()
            )

            carrot_waypoint_stamped_fixed = PoseStamped()
            carrot_waypoint_stamped_fixed.header.frame_id = self.fixed_frame
            carrot_waypoint_stamped_fixed.pose = carrot_waypoint
            # print("10.1")
            # carrot_waypoint_robot_stamped = do_transform_pose(
            #     carrot_waypoint_stamped_fixed, transform
            # )
            # print("10.2")
            # carrot_waypoint_robot = carrot_waypoint_robot_stamped.pose
            carrot_waypoint_robot = do_transform_pose(
                carrot_waypoint, transform
            )
            # print("10.2")
            carrot_waypoint_robot = carrot_waypoint_robot

        except Exception as e:
            self.get_logger().error(f'Transform error: {str(e)}')
            return False

        # print('11')
        # Publish carrot point
        ps = PoseStamped()
        # print('12')
        ps.header.frame_id = self.robot_frame
        ps.pose = carrot_waypoint_robot
        # print('13')
        self.carrot_point_pub.publish(ps)
        # print('14')

        # Calculate control commands
        carrot_x = carrot_waypoint_robot.position.x
        carrot_y = carrot_waypoint_robot.position.y
        # print('15')

        # Forward-only: rotate toward waypoints behind the robot.
        lin_vel_sign = 1
        delta_theta = math.atan2(carrot_y, carrot_x)

        ang_vel_sign = 1 if delta_theta >= 0 else -1

        carrot_dist = math.sqrt(carrot_x**2 + carrot_y**2)
        radius = carrot_dist**2 / (2.0 * (abs(carrot_y) + 0.01))
        radius_min = carrot_dist / (2.0 * math.sin(self.curvature_scaling_angle))
        radius_max = carrot_dist / (2.0 * math.sin(10.0 * math.pi / 180.0))

        if abs(delta_theta) >= self.critical_angle:
            # Case 1: Rotate in place
            self.cmd_vel.linear.x = 0.23
            self.cmd_vel.angular.z = (self.max_ang_vel) * ang_vel_sign
        elif abs(delta_theta) >= self.curvature_scaling_angle:
            # Case 2: Curvature scaled velocity
            self.cmd_vel.linear.x = lin_vel_sign * self.desired_vel * (radius / radius_min)
            self.cmd_vel.angular.z = ang_vel_sign * abs(self.cmd_vel.linear.x) / radius
            if abs(self.cmd_vel.angular.z) > self.max_ang_vel:
                self.cmd_vel.angular.z = ang_vel_sign * self.max_ang_vel
        else:
            # Case 3: Full velocity
            r = min(radius_max, radius)
            self.cmd_vel.linear.x = lin_vel_sign * self.desired_vel
            self.cmd_vel.angular.z = ang_vel_sign * abs(self.cmd_vel.linear.x) / radius
            if abs(self.cmd_vel.angular.z) > self.max_ang_vel:
                self.cmd_vel.angular.z = ang_vel_sign * self.max_ang_vel
            self.cmd_vel.linear.x *= max(0.0, (r - radius_min) / (radius_max - radius_min))

        return True


def main(args=None):
    rclpy.init(args=args)

    node = PurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
