#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random

# --- 1. FULL COORDINATE MAP ---
CITY_COORDINATES = {
    "Addis_Ababa": (0, 0), "Debre_Birhan": (2, 5), "Ambo": (-4, 1), "Adama": (4, -2),
    "Nekemte": (-8, 2), "Gimbi": (-11, 2), "Dembi_Dollo": (-14, 1), "Gambella": (-15, 0),
    "Wolkite": (-4, -2), "Jimma": (-6, -5), "Bedelle": (-8, -3), "Gore": (-10, -4),
    "Tepi": (-12, -6), "Mezan_Teferi": (-12, -8), "Bonga": (-10, -7), "Dawro": (-7, -9),
    "Wolaita_Sodo": (-4, -9), "Arba_Minch": (-4, -12), "Buta_Jirra": (0, -4), "Batu": (2, -4),
    "Worabe": (-2, -6), "Hossana": (-2, -8), "Shashemene": (0, -9), "Hawassa": (0, -11),
    "Dilla": (0, -13), "Assella": (5, -5), "Assasa": (5, -8), "Dodolla": (4, -10),
    "Bale": (6, -10), "Goba": (8, -9), "Sof_Oumer": (10, -9), "Matahara": (6, -2),
    "Awash": (8, -1), "Chiro": (10, 0), "Dire_Dawa": (12, 1), "Harar": (13, 1),
    "Babile": (14, 1), "Jigjiga": (16, 1), "Dega_Habur": (16, -3),
    "Kebri_Dehar": (18, -6), "Gode": (18, -9)
}

# --- 2. FULL ADJACENCY LIST ---
ETHIOPIA_MAP = {
    'Addis_Ababa': ['Ambo', 'Debre_Birhan', 'Adama'],
    'Ambo': ['Addis_Ababa', 'Wolkite', 'Nekemte'],
    'Debre_Birhan': ['Addis_Ababa'],
    'Adama': ['Addis_Ababa', 'Matahara', 'Batu', 'Assella'],
    'Nekemte': ['Ambo', 'Gimbi', 'Bedelle'],
    'Gimbi': ['Nekemte', 'Dembi_Dollo'],
    'Dembi_Dollo': ['Gimbi', 'Gambella'],
    'Gambella': ['Dembi_Dollo', 'Gore'],
    'Gore': ['Gambella', 'Tepi', 'Bedelle'],
    'Bedelle': ['Nekemte', 'Gore', 'Jimma'],
    'Tepi': ['Gore', 'Mezan_Teferi', 'Bonga'],
    'Mezan_Teferi': ['Tepi', 'Bonga'],
    'Bonga': ['Mezan_Teferi', 'Tepi', 'Jimma', 'Dawro'],
    'Jimma': ['Bedelle', 'Bonga', 'Wolkite'], 
    'Wolkite': ['Ambo', 'Jimma', 'Worabe'],
    'Worabe': ['Wolkite', 'Hossana', 'Buta_Jirra'],
    'Buta_Jirra': ['Worabe', 'Batu'],
    'Batu': ['Buta_Jirra', 'Adama', 'Assella'],
    'Assella': ['Adama', 'Assasa'],
    'Assasa': ['Assella', 'Dodolla'],
    'Dodolla': ['Assasa', 'Shashemene', 'Bale'],
    'Bale': ['Dodolla', 'Goba', 'Sof_Oumer'],
    'Goba': ['Bale', 'Sof_Oumer', 'Babile'], 
    'Sof_Oumer': ['Bale', 'Goba', 'Kebri_Dehar'],
    'Shashemene': ['Hossana', 'Dodolla', 'Hawassa'],
    'Hossana': ['Worabe', 'Shashemene', 'Wolaita_Sodo'],
    'Wolaita_Sodo': ['Hossana', 'Dawro', 'Arba_Minch'],
    'Dawro': ['Bonga', 'Wolaita_Sodo'],
    'Arba_Minch': ['Wolaita_Sodo'],
    'Hawassa': ['Shashemene', 'Dilla'],
    'Dilla': ['Hawassa'],
    'Matahara': ['Adama', 'Awash'],
    'Awash': ['Matahara', 'Chiro'],
    'Chiro': ['Awash', 'Dire_Dawa'],
    'Dire_Dawa': ['Chiro', 'Harar'],
    'Harar': ['Dire_Dawa', 'Babile'],
    'Babile': ['Harar', 'Jigjiga'],
    'Jigjiga': ['Babile', 'Dega_Habur'],
    'Dega_Habur': ['Jigjiga', 'Kebri_Dehar'],
    'Kebri_Dehar': ['Dega_Habur', 'Gode', 'Sof_Oumer'],
    'Gode': ['Kebri_Dehar']
}

class EthiopiaTravelNode(Node):
    def __init__(self):
        super().__init__('ethiopia_travel_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.front_dist = 10.0
        
        self.path_queue = []
        self.goal_city_coords = None
        self.goal_city_name = ""
        
        self.state = "IDLE"
        self.avoid_counter = 0  # To force duration of maneuvers
        self.avoid_state = "NONE" # BACKING, TURNING

        self.get_logger().info("Smart Travel Node (v3 - Robust Avoidance) Started!")
        self.start_mission('Addis_Ababa', 'Arba_Minch')

    def start_mission(self, start, end):
        path = self.bfs_search(start, end)
        if path:
            self.get_logger().info(f"Full Path Plan: {path}")
            self.path_queue = path[1:] # Skip start node
            self.process_next_city()
        else:
            self.get_logger().error("NO PATH FOUND!")

    def bfs_search(self, start, goal):
        queue = [[start]]
        visited = set()
        while queue:
            path = queue.pop(0)
            node = path[-1]
            if node == goal: return path
            if node not in visited:
                visited.add(node)
                for neighbor in ETHIOPIA_MAP.get(node, []):
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)
        return None

    def process_next_city(self):
        if not self.path_queue:
            self.stop_robot()
            self.state = "FINISHED"
            self.get_logger().info("MISSION SUCCESS: Destination Reached!")
            return
        
        self.goal_city_name = self.path_queue[0]
        self.goal_city_coords = CITY_COORDINATES[self.goal_city_name]
        self.get_logger().info(f">>> Next Stop: {self.goal_city_name}")
        self.state = "NAVIGATING"

    def scan_callback(self, msg):
        num_readings = len(msg.ranges)
        mid = num_readings // 2
        window = 30
        front_ranges = msg.ranges[mid - window : mid + window]
        
        valid = [r for r in front_ranges if 0.1 < r < 10.0]
        if valid:
            self.front_dist = min(valid)
        else:
            self.front_dist = 10.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.control_loop()

    def control_loop(self):
        if self.state == "FINISHED" or not self.goal_city_coords: return

        cmd = Twist()
        
        # --- LOGIC ENGINE ---
        
        # 1. OBSTACLE TRIGGER
        if self.state == "NAVIGATING" and self.front_dist < 0.8:
            self.get_logger().warn(f"Obstacle ({self.front_dist:.2f}m)! Initiating Back-up Maneuver.")
            self.state = "AVOIDING"
            self.avoid_state = "BACKING"
            self.avoid_counter = 20 # Back up for 20 cycles
        
        # 2. AVOIDANCE MANEUVER (SEQUENCE)
        if self.state == "AVOIDING":
            if self.avoid_state == "BACKING":
                if self.avoid_counter > 0:
                    cmd.linear.x = -0.3 # Move Backward
                    self.avoid_counter -= 1
                else:
                    self.get_logger().info("Backing done. Turning...")
                    self.avoid_state = "TURNING"
                    self.avoid_counter = 30 # Turn for 30 cycles
            
            elif self.avoid_state == "TURNING":
                if self.avoid_counter > 0:
                    cmd.angular.z = 0.8 # Turn Left
                    self.avoid_counter -= 1
                else:
                    # Check if clear
                    if self.front_dist > 1.2:
                        self.get_logger().info("Path clear. Resuming.")
                        self.state = "NAVIGATING"
                        self.avoid_state = "NONE"
                    else:
                        self.get_logger().warn("Still blocked. Turning more.")
                        self.avoid_counter = 20 # Turn more

            self.cmd_vel_pub.publish(cmd)
            return

        # 3. NORMAL NAVIGATION
        if self.state == "NAVIGATING":
            tx, ty = self.goal_city_coords
            dx = tx - self.current_x
            dy = ty - self.current_y
            dist = math.sqrt(dx**2 + dy**2)
            desired_yaw = math.atan2(dy, dx)
            yaw_err = desired_yaw - self.current_yaw
            
            while yaw_err > math.pi: yaw_err -= 2*math.pi
            while yaw_err < -math.pi: yaw_err += 2*math.pi

            if dist > 0.6:
                cmd.linear.x = 0.6
                cmd.angular.z = 1.5 * yaw_err
            else:
                self.get_logger().info(f"*** Arrived at {self.goal_city_name} ***")
                self.path_queue.pop(0)
                self.process_next_city()
                return

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = EthiopiaTravelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()