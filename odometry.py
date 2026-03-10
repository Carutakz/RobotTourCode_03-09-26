# odometry.py
# Tracks robot position (x, y, heading) using wheel encoders and IMU

import time
import math
import config

class Odometry:
    def __init__(self, encoders, imu, display=None):
        """
        Initialize odometry system.
        
        Args:
            encoders: Robot encoder object
            imu: Robot IMU object
            display: Optional display for debugging
        """
        self.encoders = encoders
        self.imu = imu
        self.display = display
        
        # Position state (in cm)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading in radians
        
        # Previous encoder counts
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # Robot physical parameters (from config)
        self.wheel_cm_per_count = config.WHEEL_CM_PER_COUNT  # cm per encoder tick
        self.wheelbase_cm = getattr(config, 'WHEELBASE_CM', 8.5)  # distance between wheels
        
        # IMU integration
        self.last_gyro_time = None
        self.use_imu_heading = True  # Set False to use encoder-only odometry
        
        # Initialize
        self.reset()
        
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """
        Reset odometry to a specific position.
        
        Args:
            x, y: Position in cm
            theta: Heading in radians
        """
        self.x = x
        self.y = y
        self.theta = theta
        
        # Reset encoder baseline
        left, right = self.encoders.get_counts()
        self.last_left_ticks = left
        self.last_right_ticks = right
        
        # Reset IMU time
        self.last_gyro_time = time.ticks_us()
        
        print(f"[Odometry] Reset to x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.2f}°")
    
    def update(self):
        """
        Update position estimate based on encoder and IMU data.
        Call this frequently (e.g., every 5-10ms) for accurate tracking.
        """
        # Get current encoder counts
        left_ticks, right_ticks = self.encoders.get_counts()
        
        # Calculate distance traveled by each wheel since last update
        delta_left = (left_ticks - self.last_left_ticks) * self.wheel_cm_per_count
        delta_right = (right_ticks - self.last_right_ticks) * self.wheel_cm_per_count
        
        # Update last counts
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        
        # Calculate average distance and change in heading
        delta_distance = (delta_left + delta_right) / 2.0
        
        # Update heading
        if self.use_imu_heading and self.imu.gyro.data_ready():
            # Use IMU for more accurate heading
            self.imu.gyro.read()
            turn_rate = self.imu.gyro.last_reading_dps[2]  # degrees/sec
            
            now = time.ticks_us()
            if self.last_gyro_time is not None:
                dt = time.ticks_diff(now, self.last_gyro_time) / 1_000_000
                delta_theta = math.radians(turn_rate * dt)
                self.theta += delta_theta
            self.last_gyro_time = now
        else:
            # Use encoder differential for heading (less accurate)
            delta_theta = (delta_right - delta_left) / self.wheelbase_cm
            self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = self._normalize_angle(self.theta)
        
        # Update position using current heading
        # Use midpoint integration for better accuracy
        mid_theta = self.theta - (delta_theta / 2.0 if 'delta_theta' in locals() else 0)
        
        self.x += delta_distance * math.cos(mid_theta)
        self.y += delta_distance * math.sin(mid_theta)
        
    def get_position(self):
        """Return current position as (x, y, theta) where theta is in radians."""
        return self.x, self.y, self.theta
    
    def get_position_deg(self):
        """Return current position as (x, y, theta_deg) where theta is in degrees."""
        return self.x, self.y, math.degrees(self.theta)
    
    def distance_to(self, target_x, target_y):
        """Calculate distance to a target point."""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.sqrt(dx*dx + dy*dy)
    
    def angle_to(self, target_x, target_y):
        """
        Calculate angle to target point in radians.
        Returns value in range [-pi, pi].
        """
        dx = target_x - self.x
        dy = target_y - self.y
        return math.atan2(dy, dx)
    
    def heading_error_to(self, target_x, target_y):
        """
        Calculate heading error to target point.
        Returns signed angle in radians that robot needs to turn.
        Positive = turn left, Negative = turn right
        """
        target_angle = self.angle_to(target_x, target_y)
        error = self._normalize_angle(target_angle - self.theta)
        return error
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def display_position(self):
        """Display current position on OLED (if available)."""
        if self.display:
            self.display.fill(0)
            self.display.text("Odometry", 0, 0)
            self.display.text(f"X: {self.x:6.1f} cm", 0, 15)
            self.display.text(f"Y: {self.y:6.1f} cm", 0, 30)
            self.display.text(f"θ: {math.degrees(self.theta):6.1f}°", 0, 45)
            self.display.show()
    
    def print_position(self):
        """Print current position to console."""
        x, y, theta_deg = self.get_position_deg()
        print(f"[Odometry] x={x:6.2f}cm, y={y:6.2f}cm, θ={theta_deg:6.2f}°")


class NavigationController:
    """
    High-level navigation using odometry.
    Provides methods to drive to specific coordinates.
    """
    def __init__(self, odometry, motors, turn_controller, display=None):
        self.odom = odometry
        self.motors = motors
        self.turn_controller = turn_controller
        self.display = display
        
        # Navigation parameters
        self.position_tolerance_cm = getattr(config, 'NAV_POSITION_TOLERANCE', 3.0)
        self.angle_tolerance_rad = math.radians(getattr(config, 'NAV_ANGLE_TOLERANCE', 5.0))
        self.max_speed = config.GYRO_MAX_SPEED
        self.turn_kp = 100  # Proportional gain for heading correction while driving
        
    def goto_point(self, target_x, target_y, final_heading_deg=None, 
                   backwards=False, max_iterations=1000):
        """
        Navigate to a specific point using odometry.
        
        Args:
            target_x, target_y: Target coordinates in cm
            final_heading_deg: Desired final heading in degrees (None = any heading)
            backwards: If True, drive backwards to target
            max_iterations: Safety limit on control loop iterations
            
        Returns:
            True if target reached, False if failed
        """
        print(f"[Nav] Going to ({target_x:.1f}, {target_y:.1f})")
        
        iteration = 0
        while iteration < max_iterations:
            # Update odometry
            self.odom.update()
            
            # Calculate distance and heading error
            distance = self.odom.distance_to(target_x, target_y)
            heading_error = self.odom.heading_error_to(target_x, target_y)
            
            # If driving backwards, flip the heading error
            if backwards:
                heading_error = self._normalize_angle(heading_error + math.pi)
            
            # Check if we've reached the target
            if distance < self.position_tolerance_cm:
                self.motors.off()
                print(f"[Nav] Reached target! Distance: {distance:.2f}cm")
                
                # Optionally turn to final heading
                if final_heading_deg is not None:
                    self._turn_to_heading(math.radians(final_heading_deg))
                
                return True
            
            # If heading is very wrong, stop and turn first
            if abs(heading_error) > math.radians(30):
                self.motors.off()
                self._turn_to_heading(self.odom.angle_to(target_x, target_y))
                continue
            
            # Calculate motor speeds with heading correction
            base_speed = min(self.max_speed, distance * 50)  # Slow down as we approach
            base_speed = max(base_speed, 1200)  # Minimum speed to overcome friction
            
            # Apply heading correction
            turn_correction = heading_error * self.turn_kp
            
            if backwards:
                left_speed = -(base_speed - turn_correction)
                right_speed = -(base_speed + turn_correction)
            else:
                left_speed = base_speed - turn_correction
                right_speed = base_speed + turn_correction
            
            # Clip speeds
            left_speed = self._clip_speed(left_speed)
            right_speed = self._clip_speed(right_speed)
            
            # Set motors
            self.motors.set_speeds(
                int(left_speed + config.MOTOR_LEFT_TRIM),
                int(right_speed + config.MOTOR_RIGHT_TRIM)
            )
            
            # Display progress
            if self.display and iteration % 20 == 0:
                self.display.fill(0)
                self.display.text(f"→({target_x:.0f},{target_y:.0f})", 0, 0)
                self.display.text(f"D:{distance:.1f}cm", 0, 15)
                self.display.text(f"H:{math.degrees(heading_error):.1f}°", 0, 30)
                self.display.show()
            
            iteration += 1
            time.sleep(0.01)
        
        self.motors.off()
        print(f"[Nav] Failed to reach target after {max_iterations} iterations")
        return False
    
    def follow_path(self, waypoints, final_heading_deg=None):
        """
        Follow a path defined by waypoints.
        
        Args:
            waypoints: List of (x, y) tuples in cm
            final_heading_deg: Desired heading at final waypoint
            
        Returns:
            True if all waypoints reached, False otherwise
        """
        print(f"[Nav] Following path with {len(waypoints)} waypoints")
        
        for i, (x, y) in enumerate(waypoints):
            is_last = (i == len(waypoints) - 1)
            heading = final_heading_deg if is_last else None
            
            success = self.goto_point(x, y, final_heading_deg=heading)
            if not success:
                print(f"[Nav] Failed at waypoint {i+1}/{len(waypoints)}")
                return False
            
            time.sleep(0.2)  # Brief pause between waypoints
        
        print("[Nav] Path complete!")
        return True
    
    def _turn_to_heading(self, target_heading_rad):
        """Turn to a specific absolute heading."""
        current_heading = self.odom.theta
        turn_angle = self._normalize_angle(target_heading_rad - current_heading)
        
        print(f"[Nav] Turning {math.degrees(turn_angle):.1f}° to heading {math.degrees(target_heading_rad):.1f}°")
        
        # Use the existing turn controller from motor_controls
        self.turn_controller.gyro_turn_with_line_lock(
            math.degrees(turn_angle),
            self.turn_controller.motors.imu if hasattr(self.turn_controller.motors, 'imu') else None
        )
        
        # Update odometry to match new heading
        self.odom.theta = target_heading_rad
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _clip_speed(self, speed):
        """Clip motor speed to valid range."""
        return max(-self.max_speed, min(self.max_speed, int(speed)))