#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowerPID(Node):
    def __init__(self):
        super().__init__('line_follower_pid')
        self.subscription = self.create_subscription(Image, '/front_camera', self.image_callback, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        # Base PID parameters tuned for wide projection structures
        self.Kp = 0.0048          # High immediate snap response for ninety-degree corners
        self.Kd = 0.0055          # Robust dampening factor to halt straightaway weaving
        self.Ki = 0.0000          # Zeroed out to completely eliminate memory-drag/turning-lag
        self.previous_error = 0.0
        self.current_state = 0
        
        # Dual-Tier Speed System configurations
        self.active_speed = 0.0
        self.MAX_SPEED_LIMIT = 1.5     # Full speed ceiling on clear straightaways
        self.MIN_SPEED_LIMIT = 0.20     # Hard crawl floor to maximize rotational velocity
        self.SPEED_INCREMENT = 0.030     # Rapid straight-line recovery acceleration
        self.BRAKE_DECELERATION = 0.065   # Fast deceleration when sharp turns break profile
        
        self.last_angular_z = 0.0
        self.STEERING_SMOOTH_ALPHA = 0.45 # Crisp alpha weighting for fast-acting wheel changes
        self.get_logger().info("Symmetrical Square PID Node: Predictive Multi-Row Calibration Active.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        height, width, _ = frame.shape
        camera_center = width // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        # --- DYNAMIC LOOK-AHEAD HORIZON ---
        # The faster the car is currently moving, the higher up the screen it looks.
        speed_ratio = self.active_speed / self.MAX_SPEED_LIMIT
        horizon_y = int(height * (0.70 - (speed_ratio * 0.25)))
        immediate_y = int(height * 0.85)
        
        immediate_row = binary[immediate_y, :]
        horizon_row = binary[horizon_y, :]
        
        imm_pixels = np.where(immediate_row == 255)
        hor_pixels = np.where(horizon_row == 255)
        twist = Twist()

        # STATE 0: TRACKING MODE
        if len(imm_pixels) > 0 and len(imm_pixels[0]) > 0:
            if self.current_state == 1:
                self.get_logger().info("🎯 Track Path Re-Acquired!")
                self.current_state = 0
                
            line_center = int(np.mean(imm_pixels))
            error = float(camera_center - line_center)
            
            # --- CONTINUOUS PROPORTIONAL BIAS ---
            horizon_bias = 1.0
            approaching_sharp_turn = False
            
            if len(hor_pixels) > 0 and len(hor_pixels[0]) > 0:
                horizon_center = int(np.mean(hor_pixels))
                horizon_error = float(camera_center - horizon_center)
                normalized_horizon_error = abs(horizon_error) / float(camera_center)
                
                # If the line in the distance drifts more than 10% from center
                if normalized_horizon_error > 0.10:
                    approaching_sharp_turn = True
                    # Dynamically scale the bias based on severity and current speed
                    horizon_bias = 1.0 + (normalized_horizon_error * 1.5 * (1.0 + speed_ratio))
            
            # Execute PID Math Engine with dynamic scaling adjustment
            derivative = error - self.previous_error
            raw_angular_z = float((error * self.Kp * horizon_bias) + (derivative * self.Kd))
            self.previous_error = error
            
            # Smooth steering transitions
            smoothed_angular_z = (self.STEERING_SMOOTH_ALPHA * raw_angular_z) + ((1.0 - self.STEERING_SMOOTH_ALPHA) * self.last_angular_z)
            self.last_angular_z = smoothed_angular_z
            
            # --- DYNAMIC SPEED SCALING BASED ON VELOCITY ---
            normalized_immediate_error = abs(error) / float(camera_center)
            
            # Base scaling from immediate error
            target_scaled_speed = self.MAX_SPEED_LIMIT - (normalized_immediate_error * (self.MAX_SPEED_LIMIT - self.MIN_SPEED_LIMIT))
            
            # Aggressively drop speed if a sharp turn is spotted ahead at a high velocity
            if approaching_sharp_turn:
                cornering_floor = max(self.MIN_SPEED_LIMIT, self.MAX_SPEED_LIMIT * (1.0 - normalized_horizon_error * 1.2))
                target_scaled_speed = min(target_scaled_speed, cornering_floor)
                
            target_scaled_speed = max(target_scaled_speed, self.MIN_SPEED_LIMIT)
            
            # Smoothly transition tracking step towards calculated speed target
            if self.active_speed < target_scaled_speed:
                self.active_speed = min(self.active_speed + self.SPEED_INCREMENT, target_scaled_speed)
            else:
                # Use a harder brake deceleration if we are moving fast and approaching a turn
                brake_force = self.BRAKE_DECELERATION * (1.5 if approaching_sharp_turn else 1.0)
                self.active_speed = max(self.active_speed - brake_force, target_scaled_speed)
                
            twist.linear.x = float(self.active_speed)
            twist.angular.z = float(np.clip(smoothed_angular_z, -3.2, 3.2))
            
            # Visual Feedback
            cv2.circle(frame, (line_center, immediate_y), 8, (0, 0, 255), -1)
            cv2.circle(frame, (camera_center, immediate_y), 5, (255, 0, 0), -1)
            if len(hor_pixels) > 0 and len(hor_pixels[0]) > 0:
                cv2.circle(frame, (int(np.mean(hor_pixels)), horizon_y), 6, (0, 255, 255), -1)

        # STATE 1: RECOVERY SCAN MODE
        else:
            if self.current_state == 0:
                self.get_logger().warn("🚨 Line Lost! Initiating Recovery...")
                self.current_state = 1
                
            if self.active_speed > 0.05:
                self.active_speed -= self.BRAKE_DECELERATION
            else:
                self.active_speed = 0.05
                
            twist.linear.x = float(self.active_speed)
            twist.angular.z = float(1.80) if self.previous_error >= 0 else float(-1.80)
            self.last_angular_z = twist.angular.z

        self.cmd_vel_pub.publish(twist)
        cv2.imshow("PID Line Follower Viewport", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
