import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from mavros_msgs.msg import Waypoint, State
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import json
import os
import time


class ParametrizedDroneNodeGPS(Node):
    def __init__(self):
        super().__init__('parametrized_drone_node_gps')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('namespace', '')
        self.declare_parameter('base_latitude', 38.1838)
        self.declare_parameter('base_longitude', 15.5501)
        self.declare_parameter('base_altitude', 30.0)
        self.declare_parameter('waypoint_file', '')

        self.drone_id = self.get_parameter('drone_id').value 
        self.namespace = self.get_parameter('namespace').value
        self.base_lat = self.get_parameter('base_latitude').value
        self.base_lon = self.get_parameter('base_longitude').value
        self.base_alt = self.get_parameter('base_altitude').value
        waypoint_file = self.get_parameter('waypoint_file').value

        self.get_logger().info(f"Drone {self.drone_id} started with namespace: '{self.namespace}'")
        self.get_logger().info(f"Base coordinates: {self.base_lat:.6f}, {self.base_lon:.6f}, {self.base_alt:.1f}m")

        self.flight_altitude = self.base_alt + (self.drone_id * 5.0)

        if waypoint_file and os.path.exists(waypoint_file):
            with open(waypoint_file, 'r') as f:
                all_waypoints = json.load(f)
                if self.drone_id < len(all_waypoints):
                    self.waypoints_gps = all_waypoints[self.drone_id]['waypoints']
                else: 
                    self.get_logger().error(f"Drone ID {self.drone_id} not found in waypoint file")
                    self.waypoints_gps = []
        else: 
            self.get_logger().warn("No waypoint file provided or file not found. Using default waypoints")
            self.waypoints_gps = [
                {"lat": 38.1833, "lon": 15.5511, "alt": 30.0},
                {"lat": 38.1826, "lon": 15.5505, "alt": 30.0},
                {"lat": 38.1832, "lon": 15.5496, "alt": 30.0},
                {"lat": 38.1838, "lon": 15.5515, "alt": 30.0},
                {"lat": 38.1842, "lon": 15.5508, "alt": 30.0},
                {"lat": 38.1844, "lon": 15.5502, "alt": 30.0},
                {"lat": 38.1843, "lon": 15.5493, "alt": 30.0},
                {"lat": 38.1836, "lon": 15.5492, "alt": 30.0},
                {"lat": self.base_lat, "lon": self.base_lon, "alt": self.flight_altitude},
            ]

        # Setup topic and service names
        if self.namespace:
            local_pos_topic = f'/{self.namespace}/mavros/local_position/pose'
            state_topic = f'/{self.namespace}/mavros/state'
            waypoint_push_service = f'/{self.namespace}/mavros/mission/push'
            set_mode_service = f'/{self.namespace}/mavros/set_mode'
            arming_service = f'/{self.namespace}/mavros/cmd/arming'
        else:
            local_pos_topic = 'mavros/local_position/pose'
            state_topic = 'mavros/state'
            waypoint_push_service = 'mavros/mission/push'
            set_mode_service = 'mavros/set_mode'
            arming_service = 'mavros/cmd/arming'

        self.get_logger().info(f"Using services: {waypoint_push_service}, {set_mode_service}, {arming_service}")

        # Create subscriptions
        self.local_pos_sub = self.create_subscription(PoseStamped, local_pos_topic, self.local_pos_callback, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, state_topic, self.state_callback, 10)

        # Create service clients
        self.wp_push_client = self.create_client(WaypointPush, waypoint_push_service)
        self.set_mode_client = self.create_client(SetMode, set_mode_service)
        self.arming_client = self.create_client(CommandBool, arming_service)

        self.current_state = State()
        self.current_position = None
        self.mission_sent = False
        self.mission_started = False

        # Wait for MAVROS to be ready
        self.wait_for_mavros_connection()
        
        # Execute mission sequence (like original code)
        self.wait_for_services()
        self.send_mission()
        self.arm_drone()
        self.set_auto_mission_mode()

    def wait_for_mavros_connection(self):
        """Wait for MAVROS to establish connection with PX4"""
        self.get_logger().info(f"Drone {self.drone_id}: Waiting for MAVROS connection...")
        
        # Wait for state messages to start arriving
        start_time = time.time()
        while not self.current_state.connected and (time.time() - start_time) < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.current_state.connected:
                self.get_logger().info(f"Drone {self.drone_id}: Waiting for MAVROS connection... ({time.time() - start_time:.1f}s)")
        
        if not self.current_state.connected:
            self.get_logger().error(f"Drone {self.drone_id}: Failed to connect to MAVROS after 30 seconds")
            return False
        
        self.get_logger().info(f"Drone {self.drone_id}: MAVROS connected successfully")
        return True

    def wait_for_services(self):
        """Wait for all required services to be available"""
        self.get_logger().info(f"Drone {self.drone_id}: Waiting for services...")
        
        services = [
            (self.wp_push_client, "waypoint push"),
            (self.set_mode_client, "set mode"),
            (self.arming_client, "arming")
        ]
        
        for client, service_name in services:
            timeout = 10.0
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"Drone {self.drone_id}: Service {service_name} not available after {timeout}s")
                return False
            self.get_logger().info(f"Drone {self.drone_id}: Service {service_name} available")
        
        return True

    def send_mission(self):
        """Send mission waypoints to the drone"""
        self.get_logger().info(f"Drone {self.drone_id}: Sending mission with {len(self.waypoints_gps)} waypoints...")
        
        waypoints = []
        for i, wp_gps in enumerate(self.waypoints_gps):
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16  # NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.x_lat = wp_gps["lat"]
            wp.y_long = wp_gps["lon"]
            wp.z_alt = wp_gps["alt"]
            waypoints.append(wp)
            self.get_logger().info(f"Drone {self.drone_id}: Waypoint {i+1}: ({wp.x_lat:.6f}, {wp.y_long:.6f}, {wp.z_alt:.1f}m)")

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints

        future = self.wp_push_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"Drone {self.drone_id}: Mission pushed successfully!")
            self.mission_sent = True
            return True
        else:
            self.get_logger().error(f"Drone {self.drone_id}: Error in pushing mission")
            return False

    def arm_drone(self):
        """Arm the drone"""
        self.get_logger().info(f"Drone {self.drone_id}: Attempting to arm...")
        
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f'Drone {self.drone_id}: Armed successfully')
            return True
        else:
            self.get_logger().error(f'Drone {self.drone_id}: Unable to arm')
            return False

    def set_auto_mission_mode(self):
        """Set the drone to AUTO.MISSION mode"""
        self.get_logger().info(f"Drone {self.drone_id}: Setting AUTO.MISSION mode...")
        
        req = SetMode.Request()
        req.custom_mode = "AUTO.MISSION"
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Drone {self.drone_id}: AUTO.MISSION mode successfully set!')
            return True
        else:
            self.get_logger().error(f'Drone {self.drone_id}: Error setting AUTO.MISSION mode.')
            return False

    def state_callback(self, msg):
        """Handle state updates from MAVROS"""
        if msg.mode != self.current_state.mode:
            self.get_logger().info(f"Drone {self.drone_id} - Mode changed: {msg.mode}")
        
        if msg.connected != self.current_state.connected:
            self.get_logger().info(f"Drone {self.drone_id} - Connection: {msg.connected}")
        
        if msg.armed != self.current_state.armed:
            self.get_logger().info(f"Drone {self.drone_id} - Armed: {msg.armed}")
            
        self.current_state = msg

    def local_pos_callback(self, msg):
        """Handle local position updates"""
        self.current_position = msg.pose.position


def main(args=None):
    rclpy.init(args=args)
    drone_node = ParametrizedDroneNodeGPS()
    try:
        rclpy.spin(drone_node)
    except KeyboardInterrupt:
        drone_node.get_logger().info(f"Drone {drone_node.drone_id}: Shutting down...")
    finally:
        drone_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()