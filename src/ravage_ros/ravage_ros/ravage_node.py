"""
This ROS2 node is used to inject attacks on all types of ROS compatible software by manipulating various parameters.
It parses command line arguments to determine the attack type, intensity, and software platform.
Based on these inputs, it performs the attack, logs the status, and resets the parameters to their original values.
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
import math
import yaml
import os
from datetime import datetime
import csv

# ROS2 messages, MAVROS, and services
from mavros_msgs.msg import State, StatusText, WaypointList
from mavros_msgs.srv import ParamSet, ParamGet
from sensor_msgs.msg import NavSatFix

class RavageNode(Node):
  def __init__(self):
    super().__init__('ravage_attack_engine')

    # Parameters and global variables
    self.declare_parameter('intensity', 0.0)
    self.declare_parameter('duration', 60.0)
    self.declare_parameter('attack_type', 'GPS')
    self.declare_parameter('software', 'ArduPilot')
    self.declare_parameter('config_path', './config')

    # Get initial parameters
    self.intensity = self.get_parameter('intensity').value
    self.duration = self.get_parameter('duration').value
    self.attack_type = self.get_parameter('attack_type').value.upper()
    self.software = self.get_parameter('software').value
    self.config_path = self.get_parameter('config_path').value

    # Set state variables for mission tracking
    self.current_lat = 0.0
    self.current_lon = 0.0
    self.current_alt = 0.0
    self.waypoints = []
    self.current_wp_index = 0

    # Attack variables 
    self.max_path_deviation = 0.0
    self.crash_detected = False
    self.failsafe_triggered = False
    self.attack_running = False

    # Load configs from YAML 
    self.load_configs()

    # Declare subsriptions 
    self.cb_group = ReentrantCallbackGroup() # Use ReentrantCallbackGroup to run in parallel with service calls

    self.sub_status = self.create_subscription(
      StatusText,
      '/mavros/statustext/recv',
      self.status_cb,
      10,
      callback_group=self.cb_group)

    self.sub_pos = self.create_subscription(
      NavSatFix,
      '/mavros/global_position/global',
      self.pos_cb,
      10,
      callback_group=self.cb_group)

    self.sub_mission = self.create_subscription(
      WaypointList,
      '/mavros/mission/waypoints',
      self.mission_cb,
      10,
      callback_group=self.cb_group)

    # Service clients
    self.param_set_client = self.create_client(ParamSet, '/mavros/param/set', callback_group=self.cb_group)
    self.param_get_client = self.create_client(ParamSet, '/mavros/param/get', callback_group=self.cb_group)

    while not self.param_set_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Waiting for MAVROS services...')

    self.get_logger().info(f"RAVAGE Node Initialized. Target: {self.software}, Attack: {self.attack_type}")

    # Start attack thread
    self.attack_thread = threading.Thread(target=self.runattack_sequence)
    self.attack_thread.start()
    
  def load_configs(self):
          # Load existing YAML files here
          try:
              p_conf = os.path.join(self.config_path, 'param_config.yaml')
              a_conf = os.path.join(self.config_path, 'attack_profile.yaml')
              
              with open(p_conf, 'r') as f:
                  self.param_config = yaml.safe_load(f)
              with open(a_conf, 'r') as f:
                  self.sensor_config = yaml.safe_load(f)
          except Exception as e:
              self.get_logger().error(f"Failed to load config: {e}")
              # Exit if failed

  # Subscriber callbacks
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
      # Convert MAVROS Waypoints to local list
      self.waypoints = msg.waypoints
      self.current_wp_index = msg.current_seq
      self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

  # --- Helper Methods ---

  def set_mav_param(self, param_id, value):
      """Send request to MAVROS to change a parameter"""
      req = ParamSet.Request()
      req.param_id = param_id
        
      # Determine type based on value (uses simple logic, can be expanded)
      if isinstance(value, int):
          req.value.integer = value
          req.value.real = 0.0
      else:
          req.value.integer = 0
          req.value.real = float(value)

      try:
          future = self.param_set_client.call_async(req)
          # In a thread, wait breifly for synchronization results 
          result = future.result() 
          self.get_logger().info(f"Set {param_id} to {value}")
      except Exception as e:
          self.get_logger().error(f"Failed to set param {param_id}: {e}")

  def get_mav_param(self, param_id):
      req = ParamGet.Request()
      req.param_id = param_id
      future = self.param_get_client.call_async(req)
      # Be wary not to create deadlocks
      while not future.done():
            time.sleep(0.01)
        
      try:
          res = future.result()
          if res.success:
              return res.value.integer if res.value.integer != 0 else res.value.real
      except Exception:
          return None
      return None

  def calculate_deviation(self):
      """Same logic as original"""
      if len(self.waypoints) < 2 or self.current_wp_index == 0:
          return

      # Accessing waypoints safely
      try:
          # Note: MAVROS Waypoints use .x_lat, .y_long
          wp1 = self.waypoints[self.current_wp_index - 1]
          wp2 = self.waypoints[self.current_wp_index]
            
          # Use existing logic
          dist = self.calculate_distance_to_path(
              self.current_lat, self.current_lon,
              wp1.x_lat, wp1.y_long,
              wp2.x_lat, wp2.y_long
          )
            
          if dist > self.max_path_deviation:
              self.max_path_deviation = dist
                
      except IndexError:
          pass

  def calculate_distance_to_path(self, lat, lon, wp1_lat, wp1_lon, wp2_lat, wp2_lon):
      # Reuse math function
      R = 6371000
      # Calculate bearing from wp1 to wp2
      y = math.sin(wp2_lon - wp1_lon) * math.cos(wp2_lat)
      x = math.cos(wp1_lat) * math.sin(wp2_lat) - math.sin(wp1_lat) * math.cos(wp2_lat) * math.cos(wp2_lon - wp1_lon)
      bearing_wp1_to_wp2 = math.atan2(y, x)
        
      # Calculate bearing from wp1 to current position
      y = math.sin(lon - wp1_lon) * math.cos(lat)
      x = math.cos(wp1_lat) * math.sin(lat) - math.sin(wp1_lat) * math.cos(lat) * math.cos(lon - wp1_lon)
      bearing_wp1_to_pos = math.atan2(y, x)
        
      # Calculate angular distance from wp1 to current position
      d_wp1_to_pos = math.acos(math.sin(wp1_lat) * math.sin(lat) + 
                              math.cos(wp1_lat) * math.cos(lat) * math.cos(lon - wp1_lon))
        
      # Calculate cross-track distance (XTD)
      xtd = math.asin(math.sin(d_wp1_to_pos) * math.sin(bearing_wp1_to_pos - bearing_wp1_to_wp2))
        
      # Convert to meters
      distance = abs(xtd * R)
        
      return distance

  # --- The Attack Engine ---

  def run_attack_sequence(self):
      """The main logic loop running in a separate thread"""
        
      # 1. Wait for system to settle
      time.sleep(5)
      self.get_logger().info("Starting Attack Sequence...")

      # 2. Get Parameters to Attack, update to incorporate more software
      if self.software == "ArduPilot":
          target_params = self.param_config[self.attack_type]["ArduPilot"]
      else:
          target_params = self.param_config[self.attack_type]["PX4"]

      # 3. Store Original Values
      original_values = {}
      for p in target_params:
          val = self.get_mav_param(p)
          if val is not None:
              original_values[p] = val
        
      # 4. Attack Loop
      start_time = time.time()
      current_bias = self.intensity
        
      while (time.time() - start_time < self.duration) and rclpy.ok():
          for param in target_params:
              # Inject Fault
              self.set_mav_param(param, current_bias)
            
          # Log status
          self.get_logger().info(f"Attacking {self.attack_type}: Bias {current_bias:.2f}, Deviation: {self.max_path_deviation:.2f}m")
            
          # Wait (Simulation of attack interval)
          time.sleep(1.0)

      # 5. Cleanup / Reset
      self.get_logger().info("Attack Complete. Resetting Parameters...")
      for p, val in original_values.items():
          self.set_mav_param(p, val)

      # Final Log
      self.log_attack_result()
        

  def log_attack_result(self):
      # Port your CSV logging logic here
      self.get_logger().info(f"Final Report: Max Deviation {self.max_path_deviation}")


def main(args=None):
  rclpy.init(args=args)
  node = RavageNode()
    
  # MultiThreadedExecutor might be needed if callbacks become heavy
  rclpy.spin(node)
    
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()
    




