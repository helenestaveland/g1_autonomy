#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import time



class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dt', 0.05),
                ('desired_vel', 1.0),
                ('max_lin_vel', 1.0),
                ('max_ang_vel', 1.0),
                ('max_lin_acc', 1.8),
                ('max_lin_dec', 1.8),
                ('max_ang_acc', 10.0),
                ('stop_lin_dec', 0.5),
                ('goal_reaching_threshold', 0.15),
                ('waypoint_reaching_threshold', 0.30),
                ('turn_enter_angle', math.radians(15.0)),
                ('turn_exit_angle', math.radians(5.0)),
                ('fixed_frame', 'world'),
                ('robot_frame', 'g1/base_link'),
            ]
        )

        self.load_params()

        # State variables
        self.path_to_exec = []
        self.current_pose = Pose()
        self.current_vel = Twist()
        self.cmd_vel = Twist()
        self._advance_after_stop = False
        self._settle_until = 0.0
        self._turning = False

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
        for name in (
            'dt', 'desired_vel', 'max_lin_vel', 'max_ang_vel',
            'max_lin_acc', 'max_lin_dec', 'max_ang_acc', 'stop_lin_dec',
            'goal_reaching_threshold', 'waypoint_reaching_threshold','turn_enter_angle', 'turn_exit_angle',
            'fixed_frame', 'robot_frame',
        ):
            setattr(self, name, self.get_parameter(name).value)

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
        if msg.header.frame_id and msg.header.frame_id != self.fixed_frame:
            self.get_logger().error("Path must be expressed in " + self.fixed_frame)
            self.path_to_exec = []
            return

        points = []
        for waypoint in msg.poses:
            if points and self.calculate_distance(
                    points[-1].pose, waypoint.pose) < 1e-6:
                continue

            # Remove only redundant points on the same straight segment.
            while len(points) >= 2:
                a = points[-2].pose.position
                b = points[-1].pose.position
                c = waypoint.pose.position
                ux, uy = b.x - a.x, b.y - a.y
                vx, vy = c.x - b.x, c.y - b.y
                cross = ux * vy - uy * vx
                dot = ux * vx + uy * vy
                if abs(cross) > 1e-9 or dot <= 0.0:
                    break
                points.pop()

            points.append(waypoint)

        self.path_to_exec = points
        self._advance_after_stop = False
        self._settle_until = 0.0
        self._turning = False
        self.get_logger().info(
            f"Following {len(points)} ordered path points"
        )

    def control_loop(self):
        now = time.monotonic()
        if not self.path_to_exec:
            self.stop_robot()
            return

        if self._advance_after_stop:
            self.stop_robot()

            if abs(self._last_cmd_vx) > 1e-6 or abs(self._last_cmd_wz) > 1e-6:
                self._settle_until = 0.0
                return

            if self._settle_until == 0.0:
                self._settle_until = now + 0.3
            elif now >= self._settle_until:
                if self.calculate_distance(
                        self.current_pose, self.path_to_exec[0].pose
                ) <= self.goal_reaching_threshold:
                    self.path_to_exec.pop(0)

                self._advance_after_stop = False
                self._settle_until = 0.0
                self._turning = False
            return

        if not self.compute_command_velocity():
            self.stop_robot()
            return

        vx = self.cmd_vel.linear.x
        wz = self.cmd_vel.angular.z
        previous_vx = getattr(self, '_last_cmd_vx', 0.0)
        previous_wz = getattr(self, '_last_cmd_wz', 0.0)

        # Finish braking before switching between walking and turning.
        if ((wz < 0.0 and previous_vx > 1e-6) or
                (vx > 0.0 and previous_wz < -1e-6)):
            self.stop_robot()
            self._settle_until = now + 0.3
            return

        if now < self._settle_until:
            self.stop_robot()
            return
        self._settle_until = 0.0

        rate = self.max_lin_acc if vx > previous_vx else self.max_lin_dec
        dv = rate * self.dt
        dw = self.max_ang_acc * self.dt

        self.cmd_vel.linear.x = previous_vx + max(
            -dv, min(dv, vx - previous_vx)
        )
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.z = previous_wz + max(
            -dw, min(dw, wz - previous_wz)
        )

        self._last_cmd_vx = self.cmd_vel.linear.x
        self._last_cmd_wz = self.cmd_vel.angular.z
        self.cmd_vel_pub.publish(self.cmd_vel)

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
        if not self.path_to_exec:
            return False

                # Accept nearby intermediate waypoints without stopping first.
        while len(self.path_to_exec) > 1:
            distance = self.calculate_distance(
                self.current_pose, self.path_to_exec[0].pose
            )
            if distance > self.waypoint_reaching_threshold:
                break

            self.path_to_exec.pop(0)
            self._turning = False

        waypoint = self.path_to_exec[0]
        distance = self.calculate_distance(
            self.current_pose, waypoint.pose
        )

        # Stop accurately at the final destination.
        if (len(self.path_to_exec) == 1 and
                distance <= self.goal_reaching_threshold):
            self._advance_after_stop = True
            return False

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.fixed_frame,
                rclpy.time.Time()
            )
            target = do_transform_pose(waypoint.pose, transform)
        except Exception as error:
            self.get_logger().error(f"Transform error: {error}")
            return False

        marker = PoseStamped()
        marker.header.frame_id = self.robot_frame
        marker.pose = target
        self.carrot_point_pub.publish(marker)

        angle = math.atan2(target.position.y, target.position.x)

        if self._turning:
            self._turning = abs(angle) > self.turn_exit_angle
        else:
            self._turning = abs(angle) > self.turn_enter_angle

        self.cmd_vel = Twist()

        if self._turning:
            # Always rotate right, including for targets to the left.
            self.cmd_vel.angular.z = -self.max_ang_vel
        else:
            # Walk straight and brake before reaching each corner.
            deceleration = min(self.max_lin_dec, self.stop_lin_dec)
            remaining = max(
                0.0,
                distance - 0.5 * self.goal_reaching_threshold
            )
            self.cmd_vel.linear.x = min(
                self.desired_vel,
                self.max_lin_vel,
                math.sqrt(2.0 * deceleration * remaining)
            )

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
