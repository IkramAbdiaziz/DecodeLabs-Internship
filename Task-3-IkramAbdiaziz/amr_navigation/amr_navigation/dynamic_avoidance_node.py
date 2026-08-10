import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class DynamicAvoidanceNode(Node):
    def __init__(self):
        super().__init__('dynamic_avoidance_node')
        
        # Declare parameters
        self.declare_parameter('safe_distance', 0.5)      # meters
        self.safe_dist = self.get_parameter('safe_distance').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_raw_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_raw_callback, 10)
            
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Internal State
        self.d_obstacle = float('inf')
        self.latest_raw_twist = Twist()
        self.has_raw_twist = False
        
        self.get_logger().info("Dynamic Obstacle Avoidance Node initialized.")

    def scan_callback(self, msg):
        # Isolate laser ranges in the front 60-degree sector (-30 to +30 degrees)
        front_ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Keep only front window
            if -np.radians(30) <= angle <= np.radians(30):
                if msg.range_min <= r <= msg.range_max:
                    # Exclude infinity and NaN values
                    if not np.isinf(r) and not np.isnan(r):
                        front_ranges.append(r)
                        
        if front_ranges:
            self.d_obstacle = min(front_ranges)
        else:
            self.d_obstacle = float('inf')

    def cmd_raw_callback(self, msg):
        self.latest_raw_twist = msg
        self.has_raw_twist = True
        
        # Execute safety evaluation and publish safe commands
        self.evaluate_safety()

    def evaluate_safety(self):
        if not self.has_raw_twist:
            return
            
        # Compute closing obstacle distance error
        # If no obstacle detected, error is infinity (tanh(inf) = 1.0, no slowdown)
        error = self.d_obstacle - self.safe_dist
        
        safe_twist = Twist()
        plc_trigger = 0
        
        if error <= 0.0:
            # Immediate halt
            safe_twist.linear.x = 0.0
            safe_twist.angular.z = 0.0
            plc_trigger = 1
            self.get_logger().warn(
                f"OBSTACLE INTERCEPTED! Distance: {self.d_obstacle:.2f}m <= Safe: {self.safe_dist:.2f}m. EMERGENCY STOP TRIGGERED! PLC_FAIL_TRIGGER = {plc_trigger}"
            )
        else:
            # Hyperbolic tangent deceleration override logic
            planned_vx = self.latest_raw_twist.linear.x
            
            # Apply deceleration multiplier on linear velocity only
            decel_multiplier = np.tanh(error)
            safe_twist.linear.x = planned_vx * decel_multiplier
            
            # Keep angular steering active to allow navigating around the obstacle
            safe_twist.angular.z = self.latest_raw_twist.angular.z
            
            # If deceleration is active (error is small, e.g., < 1.0m)
            if error < 1.0:
                self.get_logger().info(
                    f"Decelerating: Obstacle at {self.d_obstacle:.2f}m (Scale: {decel_multiplier:.2f}) | vx: {safe_twist.linear.x:.2f} | PLC_FAIL_TRIGGER = {plc_trigger}"
                )
                
        # Publish final safe velocity
        self.cmd_pub.publish(safe_twist)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
