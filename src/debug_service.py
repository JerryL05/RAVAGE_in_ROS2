import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamSet, ParamGet
import time

def main():
    rclpy.init()
    node = Node('debug_tester')
    
    print("1. Node created. Checking for services...")
    
    # Create Clients
    cli_get = node.create_client(ParamGet, '/mavros/param/get')
    cli_set = node.create_client(ParamSet, '/mavros/param/set')
    
    # Wait for GET
    print("2. Waiting for ParamGet...")
    while not cli_get.wait_for_service(timeout_sec=2.0):
        print("   - GET service not found yet...")
        
    print("3. SUCCESS! Found ParamGet.")
    
    # Wait for SET
    print("4. Waiting for ParamSet...")
    while not cli_set.wait_for_service(timeout_sec=2.0):
        print("   - SET service not found yet...")
        
    print("5. SUCCESS! Found ParamSet.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
