import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
import math
import yaml
import os
import csv
from datetime import datetime

# ROS2 messages
from mavros_msgs.msg import StatusText, WaypointList, OverrideRCIn
from mavros_msgs.srv import ParamSet, ParamGet
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RavageNode(Node):
    def __init__(self):
        super().__init__('ravage_attack_engine')

        # Parameters
        self.declare_parameter('intensity', 1.0)
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('attack_type', 'THROTTLE') # Should have TROTTLE, ROLL, PITCH, YAW
        self.declare_parameter('software', 'ArduPilot')
        self.declare_parameter('config_path', './config')

        self.intensity = self.get_parameter('intensity').value
        self.duration = self.get_parameter('duration').value
        self.attack_type = self.get_parameter('attack_type').value.upper()
        self.software = self.get_parameter('software').value
        self.config_path = self.get_parameter('config_path').value

        # State variables
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.waypoints = []
        self.current_wp_index = 0
        self.prev_wp_index = 0

        # Metrics and logging
        self.max_path_deviation = 0.0
        self.cumulative_deviation = 0.0
        self.deviation_samples = 0
        self.deviation_threshold = 5.0 # meters

        # Status flags
        self.status_flags = {
            "Crash": False,
            "Failsafe": False,
            "GPS_Glitch": False,
            "Bad_Mag": False,
            "Parachute": False,
            "PreArm_Error": False
        }

        self.timestamps = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_created = False

        # Load configs
        self.load_configs()

        # Reliable for sending commands
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Best effort for reading data
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Thread-safe callback group
        self.cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self.sub_status = self.create_subscription(
            StatusText, '/mavros/statustext/recv', self.status_cb, qos_best_effort, callback_group=self.cb_group)
        self.sub_pos = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.pos_cb, qos_best_effort, callback_group=self.cb_group)
        self.sub_mission = self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self.mission_cb, qos_best_effort, callback_group=self.cb_group)

        # Services
        self.param_set_client = self.create_client(ParamSet, '/mavros/param/set', callback_group=self.cb_group)
        self.param_get_client = self.create_client(ParamGet, '/mavros/param/get', callback_group=self.cb_group)

        # Publisher for RC Override
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', qos_reliable)

        self.get_logger().info(f"RAVAGE Node Initialized. Target: {self.software}, Attack: {self.attack_type}")

        # Start attack thread
        self.attack_thread = threading.Thread(target=self.run_attack_sequence)
        self.attack_thread.daemon = True # Ensure thread dies when node dies
        self.attack_thread.start()

    def load_configs(self):
        try:
            # Basic error handling for missing files
            if not os.path.exists(self.config_path):
                self.get_logger().warn(f"Config path {self.config_path} does not exist. Skipping load.")
                return

            p_conf = os.path.join(self.config_path, 'param_config.yaml')
            a_conf = os.path.join(self.config_path, 'attack_profile.yaml')

            if os.path.exists(p_conf):
                with open(p_conf, 'r') as f:
                    self.param_config = yaml.safe_load(f)
            if os.path.exists(a_conf):
                with open(a_conf, 'r') as f:
                    self.sensor_config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    # --- Callbacks ---
    def status_cb(self, msg):
        """Status porting logic"""
        text = msg.text.lower()
        self.get_logger().info(f"[MAVROS Status] {msg.text}")

        # Crash & Failsafe
        if "crash" in text or "hit ground" in text:
            self.status_flags["Crash"] = True
            self.get_logger().error("[CRITICAL] Crash Detected!")
        if "failsafe" in text or "disarmed" in text:
            self.status_flags["Failsafe"] = True

        # GPS & Sensors
        if "gps glitch" in text:
            self.status_flags["GPS_Glitch"] = True
        if "compass" in text and ("inconsistent" in text or "failure" in text):
            self.status_flags["Bad_Mag"] = True
        if "prearm" in text:
            self.status_flags["PreArm_Error"] = True
        if "parachute" in text:
            self.status_flags["Parachute"] = True

    def pos_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        self.calculate_deviation()

    def mission_cb(self, msg):
        self.waypoints = msg.waypoints
        self.prev_wp_index = self.current_wp_index
        self.current_wp_index = msg.current_seq

        if self.prev_wp_index != self.current_wp_index:
            self.get_logger().info(f"Waypoint updated: {self.current_wp_index}")

    def calculate_distance_to_path(self, lat, lon, wp1_lat, wp1_lon, wp2_lat, wp2_lon):
        """copied from original code"""
        # Convert to radians
        lat, lon = math.radians(lat), math.radians(lon)
        wp1_lat, wp1_lon = math.radians(wp1_lat), math.radians(wp1_lon)
        wp2_lat, wp2_lon = math.radians(wp2_lat), math.radians(wp2_lon)
        
        R = 6371000 # Earth radius
        
        # Bearing from wp1 to wp2
        y = math.sin(wp2_lon - wp1_lon) * math.cos(wp2_lat)
        x = math.cos(wp1_lat) * math.sin(wp2_lat) - math.sin(wp1_lat) * math.cos(wp2_lat) * math.cos(wp2_lon - wp1_lon)
        bearing_wp1_to_wp2 = math.atan2(y, x)
        
        # Bearing from wp1 to pos
        y = math.sin(lon - wp1_lon) * math.cos(lat)
        x = math.cos(wp1_lat) * math.sin(lat) - math.sin(wp1_lat) * math.cos(lat) * math.cos(lon - wp1_lon)
        bearing_wp1_to_pos = math.atan2(y, x)
        
        # Angular distance
        d_wp1_to_pos = math.acos(math.sin(wp1_lat) * math.sin(lat) + 
                                math.cos(wp1_lat) * math.cos(lat) * math.cos(lon - wp1_lon))
        
        # Cross-track distance
        xtd = math.asin(math.sin(d_wp1_to_pos) * math.sin(bearing_wp1_to_pos - bearing_wp1_to_wp2))
        return abs(xtd * R)

    def calculate_deviation(self):
        if len(self.waypoints) < 2 or self.current_wp_index == 0:
            return

        try:
            # MAVROS Waypoints use .x_lat, .y_long
            wp1 = self.waypoints[self.current_wp_index - 1]
            wp2 = self.waypoints[self.current_wp_index]
            
            dist = self.calculate_distance_to_path(
                self.current_lat, self.current_lon,
                wp1.x_lat, wp1.y_long,
                wp2.x_lat, wp2.y_long
            )
            
            self.max_path_deviation = max(self.max_path_deviation, dist)
            self.cumulative_deviation += dist
            self.deviation_samples += 1

            if dist > self.deviation_threshold:
                 self.get_logger().warn(f"[Deviation] {dist:.2f}m from path!")

        except IndexError:
            pass   

    def log_attack_status(self, final_result):
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filename = f"{log_dir}/{self.timestamp}_{self.software}_{self.attack_type}.csv"
        
        # Determine "Impact" (Binary flag if bad things happened)
        impact = 1 if (self.status_flags["Crash"] or self.status_flags["Failsafe"] or self.status_flags["GPS_Glitch"]) else 0
        crash = 1 if self.status_flags["Crash"] else 0
        
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not self.csv_file_created:
                writer.writerow(["Timestamp", "Deviation_meters", "Impact", "Crash", "Attack_result"])
                self.csv_file_created = True
            
            writer.writerow([
                datetime.now().strftime("%H:%M:%S"),
                f"{self.max_path_deviation:.4f}",
                impact,
                crash,
                final_result
            ])
            self.get_logger().info(f"Logged to {filename}")

    def get_mav_param(self, param_id):
        """Gets a parameter from the drone via MAVROS service"""
        req = ParamGet.Request()
        req.param_id = param_id
        
        # Call the service asynchronously
        future = self.param_get_client.call_async(req)
        
        # Wait for the result (Safe because we are in a separate thread)
        # We wait up to 2 seconds
        start_wait = time.time()
        while not future.done():
            if time.time() - start_wait > 2.0:
                self.get_logger().error(f"Service call timed out for {param_id}")
                return None
            time.sleep(0.01)
        
        try:
            res = future.result()
            if res.success:
                # MAVROS returns integers or floats. We check which one is non-zero/valid.
                # If integer is set, use it. Otherwise use real.
                return res.value.integer if res.value.integer != 0 else res.value.real
            else:
                self.get_logger().warn(f"Param Get failed for {param_id}")
                return None
        except Exception as e:
            self.get_logger().error(f"Service Exception: {e}")
            return None

    def set_mav_param(self, param_id, value):
        """Sets a parameter on the drone"""
        req = ParamSet.Request()
        req.param_id = param_id
        
        if isinstance(value, int):
            req.value.integer = value
            req.value.real = 0.0
        else:
            req.value.integer = 0
            req.value.real = float(value)

        future = self.param_set_client.call_async(req)
        
        # Wait briefly for result
        start_wait = time.time()
        while not future.done():
            if time.time() - start_wait > 2.0:
                 return False
            time.sleep(0.01)

        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Set {param_id} to {value}")
                return True
        except Exception:
            pass
        return False

    # --- Attack Logic ---
    def run_attack_sequence(self):
        self.get_logger().info(f"Starting attack: {self.attack_type} for {self.duration}s with intensity {self.intensity}")
        time.sleep(2.0)

        # Basic PWM values
        PWM_MIN = 1000
        PWM_MAX = 2000
        PWM_NEUTRAL = 1500

        # Calculate PWM based on Intensity
        deviation_pwm = int(500 * self.intensity) # Max deviation is 500 from neutral

        msg = OverrideRCIn()
        msg.channels = [65535] * 18 # default no override

        self.get_logger().info("Injecting attack now...")
        start_time = time.time()

        while (time.time() - start_time < self.duration) and rclpy.ok():
            if self.attack_type == "THROTTLE":
                msg.channels[2] = PWM_NEUTRAL + deviation_pwm # Go up
            elif self.attack_type == "ROLL":
                msg.channels[0] = PWM_NEUTRAL + deviation_pwm # Bank right
            elif self.attack_type == "PITCH":
                msg.channels[1] = PWM_NEUTRAL + deviation_pwm # Pitch forward
            elif self.attack_type == "YAW":
                msg.channels[3] = PWM_NEUTRAL + deviation_pwm # Spin right
            else:
                msg.channels[2] = PWM_NEUTRAL # No attack

            self.rc_pub.publish(msg)
        
            if int(time.time() * 10) % 20 == 0:
                self.get_logger().info(f"Injecting RC {self.attack_type}...")
                self.log_attack_status("running")
            
            time.sleep(0.05) # 20Hz
        # Recovery
        self.get_logger().info("Attack duration ended. Recovering controls...")
        msg.channels = [65535] * 18
        for _ in range(10):
            self.rc_pub.publish(msg)
            time.sleep(0.05)
        
        self.get_logger().info(f"Attack Complete. Max Deviation: {self.max_path_deviation:.2f}m")

        result = "success" if self.max_path_deviation > self.deviation_threshold or self.status_flags["Crash"] else "failure" 
        self.log_attack_status(result)

def main(args=None):
    rclpy.init(args=args)
    node = RavageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()