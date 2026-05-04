import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math

class SmartTurnNode(Node):
    def __init__(self):
        super().__init__('smart_turn_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.turn_offset = math.radians(90.0)
        self.target_yaw = None
        self.initial_sign = None
        
        # THE TRIPWIRE: Stop engines 5 degrees before the target to allow for coasting.
        self.coast_offset = math.radians(5.0) 
        self.turning = True

        self.get_logger().info('Tripwire Node Started. Waiting for IMU...')

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg):
        if not self.turning:
            return

        current_yaw = self.euler_from_quaternion(msg.orientation)

        if self.target_yaw is None:
            self.target_yaw = current_yaw + self.turn_offset
            self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
            
            # Calculate the initial error to know which direction we are approaching from
            initial_error = self.target_yaw - current_yaw
            initial_error = math.atan2(math.sin(initial_error), math.cos(initial_error))
            self.initial_sign = math.copysign(1, initial_error)
            
            self.get_logger().info(f'Start Yaw: {math.degrees(current_yaw):.1f}° | Bullseye Target: {math.degrees(self.target_yaw):.1f}°')
            return

        # Calculate current error
        error = self.target_yaw - current_yaw
        error = math.atan2(math.sin(error), math.cos(error))
        current_sign = math.copysign(1, error)

        # TRIPWIRE LOGIC:
        # 1. Did we get within the coasting offset?
        # 2. OR did we somehow completely blow past the target between frames (sign flipped)?
        if abs(error) <= self.coast_offset or current_sign != self.initial_sign:
            self.stop_robot(current_yaw)
        else:
            self.spin_robot(error)

    def spin_robot(self, error):
        Kp = 1.0
        cmd_speed = error * Kp

        # Clamp Maximum Speed
        if cmd_speed > 1.0: cmd_speed = 1.0
        if cmd_speed < -1.0: cmd_speed = -1.0

        # Enforce Minimum Speed so we don't get stuck on floor friction
        # Because we have a tripwire now, we WANT it to hit the wire with confidence.
        if 0 < cmd_speed < 0.2:
            cmd_speed = 0.2
        elif -0.2 < cmd_speed < 0:
            cmd_speed = -0.2

        twist = Twist()
        twist.angular.z = -cmd_speed
        self.cmd_pub.publish(twist)

    def stop_robot(self, final_yaw):
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.turning = False # Lock the node from ever sending another command
        
        final_deg = math.degrees(final_yaw)
        target_deg = math.degrees(self.target_yaw)
        
        self.get_logger().info(f'TRIPWIRE HIT! Engines cut.')
        self.get_logger().info(f'Final Yaw: {final_deg:.1f}° (Target was {target_deg:.1f}°)')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SmartTurnNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()