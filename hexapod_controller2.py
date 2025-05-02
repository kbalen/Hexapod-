#!/usr/bin/env python3
# Enhanced hexapod controller with multiple gaits and directional control

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
import time
import math
import threading
import sys
import termios
import tty

class PCA9685:
    """Simple PCA9685 PWM controller using smbus2"""
    # Register addresses
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    
    def __init__(self, bus_num, address):
        self.bus = SMBus(bus_num)
        self.address = address
        
        # Initialize
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.05)  # Wait for oscillator
        
        # Set frequency to 50Hz (for servos)
        self.set_pwm_freq(50)
    
    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescale = int(prescaleval + 0.5)
        
        oldmode = self.bus.read_byte_data(self.address, self.MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.bus.write_byte_data(self.address, self.MODE1, newmode)  # go to sleep
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, oldmode | 0xa1)
    
    def set_pwm(self, channel, on, off):
        """Sets a single PWM channel"""
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)
    
    def set_servo_angle(self, channel, angle):
        """Set servo angle (0-180 degrees)"""
        # Convert angle to pulse length (roughly 500-2500 μs)
        pulse = int(500 + (2000 * angle / 180))
        
        # Convert pulse length to 12-bit value
        value = int(pulse * 4096 / 20000)  # 20000μs = 20ms period at 50Hz
        
        # Set PWM value
        self.set_pwm(channel, 0, value)
    
    def close(self):
        self.bus.close()

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # Initialize PCA9685 boards
        try:
            # Use bus 0 (which is where i2cdetect found your devices)
            self.kit1 = PCA9685(0, 0x40)  # First controller
            self.kit2 = PCA9685(0, 0x41)  # Second controller
            
            self.get_logger().info('Servo controllers initialized')
            self.servos_enabled = True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo controllers: {e}')
            self.servos_enabled = False
        
        # Define joint to servo mapping based on your calibration script
        self.joint_to_servo_map = {
            'j1': (0, 0),    # (board_idx, channel) - Front Left Coxa
            'j2': (0, 1),    # Front Left Femur
            'j3': (0, 2),    # Front Left Tibia
            'j4': (0, 4),    # Middle Left Coxa
            'j5': (0, 5),    # Middle Left Femur
            'j6': (0, 6),    # Middle Left Tibia
            'j7': (0, 8),    # Back Left Coxa
            'j8': (0, 9),    # Back Left Femur
            'j9': (0, 15),    # Back Left Tibia
            'j10': (1, 2),   # Back Right Coxa
            'j11': (1, 1),  # Back Right Femur
            'j12': (1, 0),  # Back Right Tibia
            'j13': (1, 4),  # Middle Right Coxa
            'j14': (1, 5),  # Middle Right Femur
            'j15': (1, 6),  # Middle Right Tibia
            'j16': (1, 8),  # Front Right Coxa
            'j17': (1, 9),   # Front Right Femur (on second board)
            'j18': (1, 10)    # Front Right Tibia (on second board)
        }
        
        # Define which joints have inverted direction
        self.inverted_joints = ['j11', 'j12', 'j14', 'j15', 'j17', 'j18']
        
        # Joint limits from URDF
        self.joint_limits = {
            'j1': (-0.5, 0.5),
            'j2': (-1.0, 1.0),
            'j3': (-1.4, 0.0),
            'j4': (-0.5, 0.5),
            'j5': (-1.0, 1.0),
            'j6': (-1.4, 0.0),
            'j7': (-0.5, 0.5),
            'j8': (-1.0, 1.0),
            'j9': (-1.4, 0.0),
            'j10': (-0.5, 0.5),
            'j11': (-1.0, 1.0),
            'j12': (-1.4, 0.0),
            'j13': (-0.5, 0.5),
            'j14': (-1.0, 1.0),
            'j15': (-1.4, 0.0),
            'j16': (-0.5, 0.5),
            'j17': (-1.0, 1.0),
            'j18': (-1.4, 0.0)
        }
        
        # Mapping of leg names to joint names
        self.leg_to_joints = {
            'front_left': ['j1', 'j2', 'j3'],
            'middle_left': ['j4', 'j5', 'j6'],
            'back_left': ['j7', 'j8', 'j9'],
            'back_right': ['j10', 'j11', 'j12'],
            'middle_right': ['j13', 'j14', 'j15'],
            'front_right': ['j16', 'j17', 'j18']
        }
        
        # Load calibration
        self.calibration = {}
        try:
            from servo_calibration import calibration
            self.calibration = calibration
            self.get_logger().info('Loaded calibration from file')
        except ImportError:
            self.get_logger().info('No calibration file found, using defaults')
            for joint in self.joint_to_servo_map.keys():
                self.calibration[joint] = 0.0
        
        # Define gait patterns
        # Tripod gait - alternating groups of 3 legs
        self.tripod_groups = [
            ['front_left', 'middle_left', 'back_left'],  # Group 1
            ['front_right', 'middle_right', 'back_right']  # Group 2
        ]
        
        # Ripple gait - alternating groups of 2 legs
        self.ripple_groups = [
            ['front_left', 'back_right'],  # Group 1
            ['middle_left', 'middle_right'],  # Group 2
            ['back_left', 'front_right']  # Group 3
        ]
        
        # Wave gait - one leg at a time, front to back
        self.wave_sequence = [
            'front_right', 'middle_left', 'back_right',
            'back_left', 'middle_right', 'front_left'
        ]
        
        # Store current angles
        self.current_angles = {joint: 90.0 for joint in self.joint_to_servo_map.keys()}
        
        # Walking parameters
        self.gait_type = "tripod"  # Default gait
        self.step_height = 0.12    # Height of leg lift during step (radians)
        self.stride_angle = 0.4    # Coxa angle range during stride (radians)
        self.step_delay = 0.25     # Delay between steps (seconds)
        self.walking_direction = [0.0, 0.0]  # Default to no movement
        self.turning = 0.0         # Turning factor: -1.0 (left) to 1.0 (right)
        
        # Control flags
        self.is_walking = False
        self.stop_requested = False
        self.walking_thread = None
        
        self.get_logger().info('Hexapod Controller initialized')
    
    def rad_to_deg(self, rad):
        """Convert radians to degrees"""
        return rad * 180.0 / math.pi
    
    def deg_to_rad(self, deg):
        """Convert degrees to radians"""
        return deg * math.pi / 180.0
        
    def set_servo_angle(self, joint, angle_deg):
        """
        Set a servo to a specific angle
        
        Args:
            joint (str): Joint name
            angle_deg (float): Angle in degrees (0-180)
        """
        if not self.servos_enabled:
            self.get_logger().debug(f"Setting {joint} to {angle_deg} degrees (simulation mode)")
            return
            
        if joint not in self.joint_to_servo_map:
            self.get_logger().error(f"Unknown joint: {joint}")
            return
        
        board_idx, channel = self.joint_to_servo_map[joint]
        
        # Apply calibration offset
        servo_angle = angle_deg + self.calibration.get(joint, 0.0)
        
        # Ensure within servo range
        servo_angle = max(0, min(180, servo_angle))
        
        try:
            if board_idx == 0:
                self.kit1.set_servo_angle(channel, servo_angle)
            else:
                self.kit2.set_servo_angle(channel, servo_angle)
                
            # Update current angle
            self.current_angles[joint] = servo_angle
        except Exception as e:
            self.get_logger().error(f"Error setting servo angle for {joint}: {e}")
    
    def set_joint_angle_rad(self, joint, angle_rad):
        """
        Set a joint to a specific angle in radians, handling inverted joints and limits.

        Args:
            joint (str): Joint name
            angle_rad (float): Angle in radians
        """
        # Apply joint limits
        if joint in self.joint_limits:
            min_angle, max_angle = self.joint_limits[joint]
            angle_rad = max(min_angle, min(max_angle, angle_rad))
        
        # Convert to degrees (0-180) - neutral at 90
        angle_deg = self.rad_to_deg(angle_rad)
        
        # Apply inversion if needed
        if joint in self.inverted_joints:
            angle_deg = angle_deg
        
        # Map to servo range (0-180) - neutral at 90
        servo_angle = 90 + angle_deg
        
        # Set the servo
        self.set_servo_angle(joint, servo_angle)

    def set_leg_position(self, leg_name, angles):
        """
        Set a leg's joint angles.

        Args:
            leg_name (str): Name of the leg
            angles (list): List of 3 angles [coxa, femur, tibia] in radians
        """
        if leg_name not in self.leg_to_joints:
            self.get_logger().error(f"Unknown leg: {leg_name}")
            return
        
        if len(angles) != 3:
            self.get_logger().error(f"Expected 3 angles for leg {leg_name}, got {len(angles)}")
            return
        
        joints = self.leg_to_joints[leg_name]
        for i, joint in enumerate(joints):
            self.set_joint_angle_rad(joint, angles[i])

    def reset_all_servos(self):
        """Set all servos to neutral position (90 degrees)."""
        self.get_logger().info("Resetting all servos to neutral position")
        for joint in self.joint_to_servo_map.keys():
            self.set_servo_angle(joint, 90)
            time.sleep(0.05)  # Small delay between commands
        self.get_logger().info("All servos reset")

    def set_home_position(self):
        """Set robot to a stable standing position"""
        self.get_logger().info("Setting robot to home position")
        
        # Define standing pose angles in radians for each leg
        # [coxa, femur, tibia]
        standing_angles = {
            'front_left': [0.0, -0.5, -0.5],
            'middle_left': [0.0, -0.5, -0.5],
            'back_left': [0.0, -0.5, -0.5],
            'back_right': [0.0, -0.5, -0.5],
            'middle_right': [0.0, -0.5, -0.5],
            'front_right': [0.0, -0.5, -0.5]
        }
        
        # Apply these angles to each leg
        for leg_name, angles in standing_angles.items():
            self.set_leg_position(leg_name, angles)
            time.sleep(0.1)  # Small delay between legs
        self.get_logger().info("Home position set")
    
    def calculate_leg_step(self, leg_name, phase, direction, turn_factor=0.0):
        """
        Calculate coxa, femur, and tibia angles for a leg based on phase and direction.
        Enhanced version with improved leg lifting motion.

        Args:
            leg_name (str): Name of the leg
            phase (float): Phase of the leg's movement (0.0 to 1.0)
            direction (list): Direction vector [forward/back, left/right]
            turn_factor (float): Turn factor (-1.0 to 1.0, negative=left, positive=right)

        Returns:
            list: [coxa_angle, femur_angle, tibia_angle] in radians
        """
        # Determine if this is a left or right leg
        is_left_leg = 'left' in leg_name

        # Base modifier for turning - left and right legs need opposite coxa adjustments
        turn_modifier = 1.0 if is_left_leg else -1.0

        # Adjust stride angle and direction based on turning
        if abs(turn_factor) > 0.1:
            # If turning significantly, adjust the stride direction
            stride_for_leg = self.stride_angle * (1.0 + turn_factor * turn_modifier * 0.5)
        else:
            stride_for_leg = self.stride_angle

        # For forward/backward movement
        stride_forward = direction[0] * stride_for_leg

        # For left/right movement
        stride_side = direction[1] * stride_for_leg

        # Modified for turning - when turning, legs on one side move forward more than the other
        if turn_factor != 0.0:
            stride_forward += turn_factor * turn_modifier * self.stride_angle * 0.3

        # First half of phase (0.0-0.5): swing phase (leg in air, moving forward)
        # Second half (0.5-1.0): stance phase (leg on ground, moving backward)
        if phase < 0.5:  # Swing phase
            # Normalize to 0-1 range for swing phase
            swing_phase = phase * 2.0

            # Coxa angles for forward/backward movement
            coxa_forward = stride_forward * (swing_phase - 0.5)

            # Coxa angles for left/right movement
            coxa_side = stride_side * (swing_phase - 0.5)

            # Improved leg lifting: Use a sine curve for natural lifting motion
            lift_height = self.step_height * math.sin(swing_phase * math.pi)

            # Femur lifts leg up significantly
            femur_angle = -0.5 + lift_height * 3.0  # Multiply by 3.0 for more pronounced lift

            # Tibia compensates to keep foot position natural
            tibia_angle = -0.5 - lift_height * 2.0  # Multiply by 2.0 and invert to create natural joint movement

        else:  # Stance phase
            # Normalize to 0-1 range for stance phase
            stance_phase = (phase - 0.5) * 2.0

            # Standard stance position (all legs touching ground)
            femur_angle = -0.5
            tibia_angle = -0.5

            # Coxa angles for forward/backward movement
            coxa_forward = stride_forward * (0.5 - stance_phase)

            # Coxa angles for left/right movement
            coxa_side = stride_side * (0.5 - stance_phase)

        # Compute final coxa angle based on the leg's orientation and movement direction
        if 'front' in leg_name:
            if is_left_leg:
                # Front left: +forward, +side for left
                coxa_angle = coxa_forward + coxa_side
            else:
                # Front right: +forward, -side for right
                coxa_angle = coxa_forward - coxa_side
        elif 'middle' in leg_name:
            if is_left_leg:
                # Middle left: pure side movement
                coxa_angle = coxa_side
            else:
                # Middle right: pure side movement (negative)
                coxa_angle = -coxa_side
        else:  # 'back' in leg_name
            if is_left_leg:
                # Back left: -forward, +side for left
                coxa_angle = -coxa_forward + coxa_side
            else:
                # Back right: -forward, -side for right
                coxa_angle = -coxa_forward - coxa_side

        return [coxa_angle, femur_angle, tibia_angle]
    
    def tripod_gait_step(self, step_num, steps_per_cycle=10):
        """
        Execute one step of a tripod gait.

        Args:
            step_num (int): Current step number in the cycle
            steps_per_cycle (int): Total steps per complete gait cycle
        """
        # Calculate phase (0.0 to 1.0) within the cycle
        phase = step_num / steps_per_cycle

        # Ensure middle legs are included in the tripod groups
        self.tripod_groups = [
            ['front_left', 'back_left', 'middle_right'],  # Group 1
            ['front_right', 'middle_left', 'back_right']  # Group 2
        ]

        # For each tripod group
        for group_idx, group in enumerate(self.tripod_groups):
            # Group 0 is in phase, Group 1 is 180° out of phase (0.5 offset)
            group_phase = (phase + 0.5 * group_idx) % 1.0
            for leg_name in group:
                # Calculate leg step angles
                angles = self.calculate_leg_step(
                    leg_name, 
                    group_phase, 
                    self.walking_direction, 
                    self.turning
                )
                # Apply the calculated angles to the leg
                self.set_leg_position(leg_name, angles)
    
    def ripple_gait_step(self, step_num, steps_per_cycle=15):
        """
        Execute one step of a ripple gait (smoother, more stable than tripod).

        Args:
            step_num (int): Current step number in the cycle
            steps_per_cycle (int): Total steps per complete gait cycle
        """
        # Calculate phase (0.0 to 1.0) within the cycle
        phase = step_num / steps_per_cycle
        
        # For each ripple group (3 groups, 120° out of phase)
        for group_idx, group in enumerate(self.ripple_groups):
            # Each group is 1/3 cycle offset from the previous
            group_phase = (phase + (group_idx/3)) % 1.0
            for leg_name in group:
                angles = self.calculate_leg_step(
                    leg_name, 
                    group_phase, 
                    self.walking_direction, 
                    self.turning
                )
                self.set_leg_position(leg_name, angles)
    
    def wave_gait_step(self, step_num, steps_per_cycle=30):
        """
        Execute one step of a wave gait (one leg at a time, very stable).

        Args:
            step_num (int): Current step number in the cycle
            steps_per_cycle (int): Total steps per complete gait cycle
        """
        # Calculate phase (0.0 to 1.0) within the cycle
        phase = step_num / steps_per_cycle
        
        # For each leg in the wave sequence
        for leg_idx, leg_name in enumerate(self.wave_sequence):
            # Each leg is 1/6 cycle offset from the previous
            leg_phase = (phase + (leg_idx/6)) % 1.0
            angles = self.calculate_leg_step(
                leg_name, 
                leg_phase, 
                self.walking_direction, 
                self.turning
            )
            self.set_leg_position(leg_name, angles)
    
    def gait_step(self, step_num):
        """
        Execute one step of the current gait pattern.

        Args:
            step_num (int): Current step number
        """
        if self.gait_type == "tripod":
            self.tripod_gait_step(step_num, 10)
        elif self.gait_type == "ripple":
            self.ripple_gait_step(step_num, 15)
        elif self.gait_type == "wave":
            self.wave_gait_step(step_num, 30)
        else:
            self.get_logger().error(f"Unknown gait type: {self.gait_type}")
    
    def walking_loop(self):
        """Main walking loop, runs in a separate thread"""
        self.get_logger().info(f"Starting walking using {self.gait_type} gait")
        
        step_num = 0
        self.stop_requested = False
        while not self.stop_requested:
            self.gait_step(step_num)
            step_num += 1
            time.sleep(self.step_delay)
        self.is_walking = False
        self.get_logger().info("Walking stopped")
    
    def start_walking(self):
        """Start the walking loop in a separate thread"""
        if self.is_walking:
            self.get_logger().info("Already walking, ignoring request")
            return
        
        self.is_walking = True
        self.stop_requested = False
        
        # Start walking in a separate thread
        self.walking_thread = threading.Thread(target=self.walking_loop)
        self.walking_thread.daemon = True
        self.walking_thread.start()
    
    def stop_walking(self):
        """Request the walking loop to stop"""
        if not self.is_walking:
            return
        
        self.get_logger().info("Stopping walking...")
        self.stop_requested = True
        
        # Wait for the walking thread to finish
        if self.walking_thread and self.walking_thread.is_alive():
            self.walking_thread.join(timeout=2.0)
        
        # Return to home position
        self.set_home_position()
    
    def set_gait(self, gait_type):
        """
        Set the gait pattern to use.

        Args:
            gait_type (str): One of "tripod", "ripple", or "wave"
        """
        if gait_type not in ["tripod", "ripple", "wave"]:
            self.get_logger().error(f"Unknown gait type: {gait_type}")
            return
        
        self.gait_type = gait_type
        self.get_logger().info(f"Set gait to {gait_type}")
    
    def set_direction(self, forward, side, turn=0.0):
        """
        Set walking direction vector.

        Args:
            forward (float): Forward/back component (-1.0 to 1.0)
            side (float): Left/right component (-1.0 to 1.0)
            turn (float): Turning component (-1.0 to 1.0, negative=left, positive=right)
        """
        # Normalize direction vector
        mag = math.sqrt(forward**2 + side**2)
        if mag > 1.0:
            forward /= mag
            side /= mag
        
        self.walking_direction = [forward, side]
        self.turning = max(-1.0, min(1.0, turn))
        self.get_logger().info(f"Set direction: forward={forward:.1f}, side={side:.1f}, turn={turn:.1f}")
    
    def set_speed(self, step_delay):
        """
        Set walking speed by adjusting step delay.

        Args:
            step_delay (float): Delay between steps in seconds (smaller = faster)
        """
        self.step_delay = max(0.05, min(0.5, step_delay))
        self.get_logger().info(f"Set step delay to {self.step_delay:.2f}s")
    
    def print_help(self):
        """Print help information"""
        print("\nHexapod Controller Commands:")
        print("----------------------------")
        print("w/a/s/d - Move forward/left/backward/right")
        print("q/e     - Turn left/right")
        print("1/2/3   - Set gait (tripod/ripple/wave)")
        print("+/-     - Increase/decrease speed")
        print("h       - Return to home position")
        print("space   - Start/stop walking")
        print("?       - Show this help")
        print("x       - Exit")
        print("")
    
    def get_key(self):
        """Get a single keypress from the terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def interactive_control(self):
        """Interactive control loop using keyboard input"""
        self.print_help()
        while True:
            key = self.get_key().lower()
            if key == 'x':  # Exit
                print("Exiting...")
                if self.is_walking:
                    self.stop_walking()
                break
            elif key == 'w':  # Forward
                self.set_direction(1.0, 0.0, 0.0)
            elif key == 's':  # Backward
                self.set_direction(-1.0, 0.0, 0.0)
            elif key == 'a':  # Left
                self.set_direction(0.0, 1.0, 0.0)
            elif key == 'd':  # Right
                self.set_direction(0.0, -1.0, 0.0)
            elif key == 'q':  # Turn left
                self.set_direction(0.0, 0.0, -1.0)
            elif key == 'e':  # Turn right
                self.set_direction(0.0, 0.0, 1.0)
            elif key == '1':  # Tripod gait
                self.set_gait("tripod")
            elif key == '2':  # Ripple gait
                self.set_gait("ripple")
            elif key == '3':  # Wave gait
                self.set_gait("wave")
            elif key == '+' or key == '=':  # Faster
                self.set_speed(self.step_delay * 0.8)
            elif key == '-' or key == '_':  # Slower
                self.set_speed(self.step_delay * 1.2)
            elif key == 'h':  # Home position
                self.set_home_position()
            elif key == ' ':  # Start/stop walking
                if self.is_walking:
                    self.stop_walking()
                else:
                    self.start_walking()
            elif key == '?':  # Help
                self.print_help()

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    try:
        # First reset all servos
        node.reset_all_servos()
        time.sleep(1)
        
        # Set to home position
        node.set_home_position()
        time.sleep(2)
        
        # Start interactive control
        node.interactive_control()
    except KeyboardInterrupt:
        node.get_logger().info("Controller interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        # Make sure to set home position before exiting
        node.set_home_position()
        time.sleep(1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()