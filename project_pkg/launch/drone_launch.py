# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def launch_setup(context, *args, **kwargs):
#     num_drones = int(LaunchConfiguration('num_drones').perform(context))
#     nodes = []
    
#     for i in range(num_drones):
#         namespace = f"drone{i}" if num_drones>1 else ""
#         fcu_url = f"udp://:{14540 + i}@127.0.0.1:{14580 + i}"
#         target_system = i + 1
        
#         # MAVROS Node
#         mavros_node = Node(
#             package='mavros',
#             executable='mavros_node',
#             namespace=namespace,
#             name=f'mavros_{i}',
#             output='screen',
#             parameters=[
#                 {'fcu_url': fcu_url},
#                 {'target_system_id': target_system},
#                 {'target_component_id': 1},
#                 {'system_id': target_system},
#                 {'component_id': 1},
#                 {'use_sim_time': True},
#             ]
#         )
#         nodes.append(mavros_node)
        
#         # Your Drone Node
#         drone_node_delayed = TimerAction(
#         	period=10.0,
#         	actions=[
# 			Node(
# 			    package='project_pkg',
# 			    executable='swarm',
# 			    namespace=namespace,
# 			    name=f'drone_controller_{i}',
# 			    output='screen',
# 			    parameters=[
# 				{'drone_id': i},
# 				{'namespace': namespace},
# 				{'base_latitude': 38.1838},
# 				{'base_longitude': 15.5501},
# 				{'base_altitude': 30.0},
# 				{'waypoint_file': '/home/user/ros2_ws/src/Project_PX4_IIoT/project_pkg/waypoint_file.json'},  # Opzionale
# 			    ]
# 			)
# 		]
# 	)
#         nodes.append(drone_node_delayed)
    
#     return nodes

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('num_drones', default_value='1'),
#         OpaqueFunction(function=launch_setup)
#     ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def launch_setup(context, *args, **kwargs):
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    nodes = []
 
    for i in range(num_drones):
        namespace = f"drone{i}"
        fcu_url = f"udp://:{14540 + i}@127.0.0.1:{14580 + i}"
        target_system = str(i + 1)
 
        node = Node(
            package='mavros',
            executable='mavros_node',
            namespace=namespace,
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': fcu_url},
                {'target_system_id': target_system},
                {'target_component_id': '1'},
                {'system_id': target_system},
                {'component_id': '1'},
                {'use_sim_time': True},
            ]
        )
        nodes.append(node)
 
    return nodes
 
 
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='2'),
        OpaqueFunction(function=launch_setup)
    ])

