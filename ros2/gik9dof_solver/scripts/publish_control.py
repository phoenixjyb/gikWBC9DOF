import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        default_path = Path(__file__).resolve().with_name('camera_traj_world.json')
        self.declare_parameter('trajectory_path', str(default_path))

        file_path_value = self.get_parameter('trajectory_path').get_parameter_value().string_value
        file_path = Path(file_path_value).expanduser()

        if not file_path.exists():
            raise FileNotFoundError(f'Cannot find trajectory JSON at {file_path}')

        with file_path.open('r', encoding='utf-8') as file:
            data = json.load(file)

        self.positions = [pose['position'] for pose in data['poses']]
        self.orientations = [pose['orientation'] for pose in data['poses']]
        self.yaws = [self.quaternion_to_yaw(q) for q in self.orientations]

        if len(self.positions) < 2:
            raise ValueError('Trajectory must contain at least two poses')

        self.current_index = 1

        self.linear_velocity_scale = 1.0
        self.angular_velocity_scale = 1.0

        self.publisher = self.create_publisher(Twist, '/mobile_base/commands/velocity', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'Velocity publisher started at 10 Hz using {file_path}')
        self.get_logger().info(f'Linear velocity scale: {self.linear_velocity_scale}x')
        self.get_logger().info(f'Angular velocity scale: {self.angular_velocity_scale}x')

    def timer_callback(self):
        if self.current_index >= len(self.positions):
            self.get_logger().info('All velocities published. Stopping.')
            self.timer.cancel()
            return

        x1, y1, _ = self.positions[self.current_index - 1]
        x2, y2, _ = self.positions[self.current_index]

        yaw1 = self.yaws[self.current_index - 1]
        yaw2 = self.yaws[self.current_index]

        dt = 0.1  # 10 Hz

        yaw_delta = self.normalize_angle(yaw2 - yaw1)
        w_z = yaw_delta / dt

        delta_x_world = x2 - x1
        delta_y_world = y2 - y1

        cos_yaw = math.cos(yaw1)
        sin_yaw = math.sin(yaw1)

        delta_x_body = cos_yaw * delta_x_world + sin_yaw * delta_y_world
        delta_y_body = -sin_yaw * delta_x_world + cos_yaw * delta_y_world

        v_x = delta_x_body / dt

        twist = Twist()
        twist.linear.x = v_x * self.linear_velocity_scale
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = w_z * self.angular_velocity_scale

        self.publisher.publish(twist)
        self.get_logger().info(
            f"[{self.current_index}/{len(self.positions) - 1}] "
            f"v_x={twist.linear.x:.3f} m/s, w_z={twist.angular.z:.3f} rad/s "
            f"(dy_body={delta_y_body:.4f} m)"
        )

        self.current_index += 1

    @staticmethod
    def quaternion_to_yaw(q):
        """Extract yaw (rotation about Z) from quaternion [x, y, z, w]."""
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
