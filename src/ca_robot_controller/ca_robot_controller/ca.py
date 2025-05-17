import rclpy
from rclpy.node import Node
import numpy as np
import random

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class CANavigator(Node):
    def __init__(self):
        super().__init__('ca_navigator')

        self.subscriber = self.create_subscription(
            LaserScan, '/gazebo_ros_ray_sensor/out', self.lidar_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/ca_local_grid', 10)

        self.grid_size = 30
        self.cell_size = 0.2
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)

        self.timer = self.create_timer(0.1, self.navigate)

        # Κατάσταση escape
        self.stuck_counter = 0
        self.escape_mode = False
        self.escape_steps = 0

    def lidar_callback(self, msg):
        self.grid.fill(0)
        angle = msg.angle_min

        for r in msg.ranges:
            if 0.1 < r < msg.range_max:
                x = int((r * np.cos(angle)) / self.cell_size + self.grid_size // 2)
                y = int((-r * np.sin(angle)) / self.cell_size + self.grid_size // 2)
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    self.grid[x, y] = 1
            angle += msg.angle_increment

    def count_neighbors(self, x, y):
        count = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    count += self.grid[nx, ny]
        return count

    def update_grid_ca(self):
        new_grid = np.copy(self.grid)
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                neighbors = self.count_neighbors(x, y)
                if self.grid[x, y] == 1:
                    # Κανόνας: αν έχει λιγότερους από 2 γείτονες, "πεθαίνει" (γίνεται 0)
                    if neighbors < 2:
                        new_grid[x, y] = 0
                else:
                    # Κανόνας: αν έχει πάνω από 4 γείτονες, γίνεται εμπόδιο (1)
                    if neighbors > 4:
                        new_grid[x, y] = 1
        self.grid = new_grid

    def navigate(self):
        # Ενημέρωση πλέγματος μέσω CA
        self.update_grid_ca()

        center = self.grid_size // 2
        search_range = 5
        danger_threshold = 3
        twist = Twist()

        forward_region = self.grid[center + 1:center + search_range, center - 2:center + 3]
        forward_obstacles = np.sum(forward_region)
        total_cells = forward_region.size

        # Αν έχει πολλά εμπόδια μπροστά, ανέβασε counter
        if forward_obstacles > total_cells * 0.8:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        # Ενεργοποίηση escape mode
        if self.stuck_counter > 15 and not self.escape_mode:
            self.get_logger().warn("🚨 Παγίδευση εντοπίστηκε: ενεργοποίηση escape mode")
            self.escape_mode = True
            self.escape_steps = 10
            self.stuck_counter = 0

        if self.escape_mode:
            twist.linear.x = -0.1
            twist.angular.z = random.choice([-0.8, 0.8])
            self.escape_steps -= 1

            # Βγες από escape mode μόνο αν βλέπει καθαρά μπροστά
            if self.escape_steps <= 0 and forward_obstacles < 3:
                self.get_logger().info("✅ Έξοδος από escape mode")
                self.escape_mode = False

        else:
            if forward_obstacles > danger_threshold:
                left = np.sum(self.grid[center - 2:center + 3, center + 2:center + 6])
                right = np.sum(self.grid[center - 2:center + 3, center - 6:center - 2])
                twist.angular.z = -0.5 if left < right else 0.5
            else:
                twist.linear.x = 0.2

        self.cmd_pub.publish(twist)
        self.publish_occupancy_grid()

    def publish_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size

        offset = (self.grid_size * self.cell_size) / 2.0
        msg.info.origin.position.x = -offset
        msg.info.origin.position.y = -offset
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        flipped = np.flipud(self.grid.T)
        msg.data = [100 if val == 1 else 0 for val in flipped.flatten()]
        self.grid_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = CANavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

