import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import heapq

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        
        # Declare parameters
        self.declare_parameter('lookahead_distance', 0.35)  # meters
        self.declare_parameter('control_frequency', 10.0)    # Hz
        self.declare_parameter('inflation_cells', 4)         # cell margin
        
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.inflation_cells = self.get_parameter('inflation_cells').value
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
            
        # Publishers
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Internal State
        self.map_data = None
        self.map_info = None
        self.map_grid = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        
        self.goal_x = None
        self.goal_y = None
        self.global_path_w = [] # List of (x, y) world coordinates
        
        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, self.control_loop)
            
        self.get_logger().info("A* Path Planner Node initialized.")

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = msg.data
        
        # Convert flat list to 2D numpy array (row-major order)
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        
        # Inflate obstacles to prevent chassis scraping
        self.map_grid = np.copy(grid)
        rows, cols = np.where(grid == 100)
        for r, c in zip(rows, cols):
            r_min = max(0, r - self.inflation_cells)
            r_max = min(msg.info.height, r + self.inflation_cells + 1)
            c_min = max(0, c - self.inflation_cells)
            c_max = min(msg.info.width, c + self.inflation_cells + 1)
            self.map_grid[r_min:r_max, c_min:c_max] = 100
            
        self.get_logger().info(f"Loaded and inflated map: {msg.info.width}x{msg.info.height} cells.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def goal_callback(self, msg):
        if self.map_grid is None or not self.has_odom:
            self.get_logger().warn("Cannot plan: Map or Odometry is missing!")
            return
            
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        
        # Plan path using A*
        self.plan_path()

    def world_to_grid(self, x, y):
        # Convert continuous coordinates to discrete cells
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return row, col

    def grid_to_world(self, row, col):
        # Convert discrete cells to continuous coordinates (center of the cell)
        x = col * self.map_info.resolution + self.map_info.origin.position.x + 0.5 * self.map_info.resolution
        y = row * self.map_info.resolution + self.map_info.origin.position.y + 0.5 * self.map_info.resolution
        return x, y

    def heuristic(self, a, b):
        # Manhattan Distance Heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan_path(self):
        start_row, start_col = self.world_to_grid(self.current_x, self.current_y)
        goal_row, goal_col = self.world_to_grid(self.goal_x, self.goal_y)
        
        height, width = self.map_grid.shape
        
        # Bounds check
        if not (0 <= start_row < height and 0 <= start_col < width):
            self.get_logger().error("Robot start position is outside the map boundaries!")
            return
        if not (0 <= goal_row < height and 0 <= goal_col < width):
            self.get_logger().error("Goal position is outside the map boundaries!")
            return
            
        # Check if start or goal is in collision
        if self.map_grid[start_row, start_col] >= 50:
            self.get_logger().warn("Start pose is in inflated collision zone. Forcing search...")
        if self.map_grid[goal_row, goal_col] >= 50:
            self.get_logger().error("Goal pose is in occupied collision zone! Planning aborted.")
            return

        self.get_logger().info(f"Planning from cell ({start_row}, {start_col}) to ({goal_row}, {goal_col})")
        
        # 8-connected neighborhood movements (direction_row, direction_col, movement_cost)
        directions = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),   # Straight
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414) # Diagonal
        ]
        
        start = (start_row, start_col)
        goal = (goal_row, goal_col)
        
        # Priority Queue: (f_score, g_score, cell)
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), 0.0, start))
        
        came_from = {}
        g_score = {start: 0.0}
        
        found = False
        while open_set:
            f, g, current = heapq.heappop(open_set)
            
            if current == goal:
                found = True
                break
                
            for dr, dc, cost in directions:
                neighbor = (current[0] + dr, current[1] + dc)
                
                if not (0 <= neighbor[0] < height and 0 <= neighbor[1] < width):
                    continue
                    
                # Obstacle check: cell value >= 50 represents occupied or inflated collision
                # -1 represents unknown space (treated as obstacle for safety)
                if self.map_grid[neighbor[0], neighbor[1]] >= 50 or self.map_grid[neighbor[0], neighbor[1]] == -1:
                    continue
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        if found:
            # Reconstruct path
            grid_path = []
            curr = goal
            while curr in came_from:
                grid_path.append(curr)
                curr = came_from[curr]
            grid_path.append(start)
            grid_path.reverse()
            
            # Convert to world coordinates
            self.global_path_w = [self.grid_to_world(r, c) for r, c in grid_path]
            self.get_logger().info(f"Global plan successfully generated with {len(self.global_path_w)} waypoints.")
            
            # Publish global plan visual path
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for wx, wy in self.global_path_w:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
                
            self.path_pub.publish(path_msg)
        else:
            self.get_logger().error("A* search failed: No path could be resolved to the goal pose!")
            self.global_path_w = []

    def control_loop(self):
        if not self.global_path_w or not self.has_odom:
            return
            
        # Target coordinate tracking
        # Find the point along the path that is at least lookahead_dist away
        target_pt = None
        for wx, wy in self.global_path_w:
            dist = np.hypot(wx - self.current_x, wy - self.current_y)
            if dist >= self.lookahead_dist:
                target_pt = (wx, wy)
                break
                
        # If all remaining points are within lookahead, target the final goal
        if target_pt is None:
            target_pt = self.global_path_w[-1]
            
        dx = target_pt[0] - self.current_x
        dy = target_pt[1] - self.current_y
        dist_to_target = np.hypot(dx, dy)
        dist_to_goal = np.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        
        twist = Twist()
        
        # Check if we have arrived at the goal
        if dist_to_goal < 0.15:
            self.get_logger().info("Target goal achieved! Halting robot.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.global_path_w = []  # Clear plan
        else:
            # Steering controller calculation
            target_yaw = np.arctan2(dy, dx)
            heading_err = target_yaw - self.current_yaw
            
            # Normalize error to [-pi, pi]
            heading_err = np.arctan2(np.sin(heading_err), np.cos(heading_err))
            
            if abs(heading_err) > 0.8:
                # Rotate in place first if orientation error is large
                self.get_logger().info("Large heading error. Rotating in place...")
                twist.linear.x = 0.0
                twist.angular.z = 1.0 * np.sign(heading_err)
            else:
                # Proportional steering
                twist.linear.x = 0.25  # m/s
                twist.angular.z = 2.0 * heading_err
                
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
