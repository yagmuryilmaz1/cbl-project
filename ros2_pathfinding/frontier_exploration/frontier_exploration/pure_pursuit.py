#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import numpy as np
from path_planner import PathPlanner
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Path, Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3, Pose, Quaternion
import tf_transformations
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException


class PurePursuit(Node):
    def __init__(self):
        """
        Class constructor
        """
        super().__init__('pure_pursuit')

        # Set if in debug mode
        self.declare_parameter('debug', False)
        self.is_in_debug_mode = self.get_parameter('debug').value

        # Publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lookahead_pub = self.create_publisher(
            PointStamped, '/pure_pursuit/lookahead', 10
        )

        if self.is_in_debug_mode:
            self.fov_cells_pub = self.create_publisher(
                GridCells, '/pure_pursuit/fov_cells', 100
            )
            self.close_wall_cells_pub = self.create_publisher(
                GridCells, '/pure_pursuit/close_wall_cells', 100
            )

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.create_subscription(OccupancyGrid, '/map', self.update_map, 10)
        self.create_subscription(Path, '/pure_pursuit/path', self.update_path, 10)
        self.create_subscription(Bool, '/pure_pursuit/enabled', self.update_enabled, 10)

        # Pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = 0.18  # m
        self.WHEEL_BASE = 0.16  # m
        self.MAX_DRIVE_SPEED = 0.1  # m/s
        self.MAX_TURN_SPEED = 1.25  # rad/s
        self.TURN_SPEED_KP = 1.25
        self.DISTANCE_TOLERANCE = 0.1  # m

        # Obstacle avoidance parameters
        self.OBSTACLE_AVOIDANCE_GAIN = 0.3
        self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE = 0.16  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE = 0.12  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR = 0.25
        self.FOV = 200  # degrees
        self.FOV_DISTANCE = 25  # Number of grid cells
        self.FOV_DEADZONE = 80  # degrees
        self.SMALL_FOV = 300  # degrees
        self.SMALL_FOV_DISTANCE = 10  # Number of grid cells

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose = None
        self.map = None
        self.path = Path()
        self.alpha = 0
        self.enabled = True
        self.reversed = False
        self.closest_distance = float("inf")

    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            trans = transform.transform.translation
            rot = transform.transform.rotation
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')
            return

        self.pose = Pose(
            position=Point(x=trans.x, y=trans.y),
            orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
        )

    def update_map(self, msg: OccupancyGrid):
        """
        Updates the current map.
        This method is a callback bound to a Subscriber.
        :param msg [OccupancyGrid] The current map information.
        """
        self.map = msg

    def update_path(self, msg: Path):
        self.path = msg

    def update_enabled(self, msg: Bool):
        self.enabled = msg.data

    def calculate_steering_adjustment(self) -> float:
        if self.pose is None or self.map is None:
            return 0

        orientation = self.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        yaw = float(np.rad2deg(yaw))

        # Get the grid cell of the robot
        robot_cell = PathPlanner.world_to_grid(self.map, self.pose.position)

        weighted_sum_of_angles = 0
        total_weight = 0
        self.closest_distance = float("inf")

        # Get all wall cells near the robot within the distance
        fov_cells = []
        wall_cells = []
        wall_cell_count = 0
        for dx in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
            for dy in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
                cell = (robot_cell[0] + dx, robot_cell[1] + dy)
                distance = PathPlanner.euclidean_distance(robot_cell, cell)

                # If the cell is out of bounds, ignore it
                if not PathPlanner.is_cell_in_bounds(self.map, cell):
                    continue

                is_wall = not PathPlanner.is_cell_walkable(self.map, cell)
                if is_wall and distance < self.closest_distance:
                    self.closest_distance = distance

                # Calculate the angle of the cell relative to the robot
                angle = float(np.rad2deg(np.arctan2(dy, dx))) - yaw

                # If reversed, add 180 to the angle
                if self.reversed:
                    angle += 180

                # Keep angle in the range of -180 to 180
                if angle < -180:
                    angle += 360
                elif angle > 180:
                    angle -= 360

                # Ignore scans that are outside the field of view
                is_in_fov = (
                    distance <= self.FOV_DISTANCE
                    and angle >= -self.FOV / 2
                    and angle <= self.FOV / 2
                    and not abs(angle) < self.FOV_DEADZONE / 2
                )
                is_in_small_fov = (
                    distance <= self.SMALL_FOV_DISTANCE
                    and angle >= -self.SMALL_FOV / 2
                    and angle <= self.SMALL_FOV / 2
                )
                if not is_in_fov and not is_in_small_fov:
                    continue

                # If in debug mode, add the cell to the field of view
                if self.is_in_debug_mode:
                    fov_cells.append(cell)

                # If cell is not a wall, ignore it
                if not is_wall:
                    continue

                weight = 1 / (distance**2) if distance != 0 else 0

                weighted_sum_of_angles += weight * angle
                total_weight += weight

                wall_cell_count += 1

                if self.is_in_debug_mode:
                    wall_cells.append(cell)

        # If in debug mode, publish the wall cells
        if self.is_in_debug_mode:
            self.fov_cells_pub.publish(PathPlanner.get_grid_cells(self.map, fov_cells))
            self.close_wall_cells_pub.publish(
                PathPlanner.get_grid_cells(self.map, wall_cells)
            )

        if total_weight == 0:
            return 0

        # Calculate the average angle (weighted sum of angles divided by total weight)
        average_angle = weighted_sum_of_angles / total_weight

        # Calculate the steering adjustment based on the average angle
        steering_adjustment = (
            -self.OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count
        )

        return steering_adjustment

    @staticmethod
    def distance(x0, y0, x1, y1) -> float:
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def get_distance_to_waypoint_index(self, i: int) -> float:
        if self.pose is None or len(self.path.poses) == 0:
            return float("inf")

        waypoint = self.path.poses[i].pose.position
        return PurePursuit.distance(
            self.pose.position.x,
            self.pose.position.y,
            waypoint.x,
            waypoint.y,
        )

    def find_nearest_waypoint_index(self) -> int:
        if self.pose is None or len(self.path.poses) == 0:
            return 0

        min_distance = float("inf")
        min_index = 0

        for i in range(len(self.path.poses)):
            distance = self.get_distance_to_waypoint_index(i)
            if distance < min_distance:
                min_distance = distance
                min_index = i

        return min_index

    def find_lookahead(self, nearest_waypoint_index, lookahead_distance) -> Point:
        if self.pose is None or len(self.path.poses) == 0:
            return Point()

        for i in range(nearest_waypoint_index, len(self.path.poses)):
            waypoint = self.path.poses[i].pose.position
            distance = PurePursuit.distance(
                self.pose.position.x,
                self.pose.position.y,
                waypoint.x,
                waypoint.y,
            )
            if distance >= lookahead_distance:
                return waypoint

        return self.path.poses[-1].pose.position

    def get_goal(self) -> Point:
        if self.pose is None or len(self.path.poses) == 0:
            return Point()

        nearest_waypoint_index = self.find_nearest_waypoint_index()
        lookahead = self.find_lookahead(nearest_waypoint_index, self.LOOKAHEAD_DISTANCE)

        return lookahead

    def send_speed(self, linear_speed: float, angular_speed: float):
        if not self.enabled:
            return

        twist = Twist()
        twist.linear = Vector3(x=linear_speed, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=angular_speed)
        self.cmd_vel.publish(twist)

    def stop(self):
        self.send_speed(0, 0)

    def run(self):
        rate = self.create_rate(20)  # Hz
        while rclpy.ok():
            if self.pose is None or self.map is None or len(self.path.poses) == 0:
                rate.sleep()
                continue

            # Get the goal point
            goal = self.get_goal()

            # Calculate the distance to the goal
            distance = PurePursuit.distance(
                self.pose.position.x,
                self.pose.position.y,
                goal.x,
                goal.y,
            )

            # If we're close enough to the goal, stop
            if distance < self.DISTANCE_TOLERANCE:
                self.stop()
                rate.sleep()
                continue

            # Calculate the angle to the goal
            angle = math.atan2(
                goal.y - self.pose.position.y, goal.x - self.pose.position.x
            )

            # Get the current orientation
            orientation = self.pose.orientation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )

            # Calculate the angle difference
            angle_diff = angle - yaw

            # Keep angle difference in the range of -pi to pi
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Calculate the steering adjustment
            steering_adjustment = self.calculate_steering_adjustment()

            # Calculate the angular speed
            angular_speed = self.TURN_SPEED_KP * angle_diff + steering_adjustment

            # Limit the angular speed
            if angular_speed > self.MAX_TURN_SPEED:
                angular_speed = self.MAX_TURN_SPEED
            elif angular_speed < -self.MAX_TURN_SPEED:
                angular_speed = -self.MAX_TURN_SPEED

            # Calculate the linear speed
            linear_speed = self.MAX_DRIVE_SPEED

            # Slow down if we're close to a wall
            if self.closest_distance < self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE:
                slow_down_factor = 1.0
                if self.closest_distance < self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE:
                    slow_down_factor = self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR
                else:
                    slow_down_factor = (
                        self.closest_distance
                        - self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE
                    ) / (
                        self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE
                        - self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE
                    )
                linear_speed *= slow_down_factor

            # Send the speed
            self.send_speed(linear_speed, angular_speed)

            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()