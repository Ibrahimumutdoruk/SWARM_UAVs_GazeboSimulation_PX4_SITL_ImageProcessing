"""
Madde 3 launch: profile-driven config.
  ros2 launch laplacian_swarm swarm_bringup.launch.py            # = sitl (default)
  ros2 launch laplacian_swarm swarm_bringup.launch.py profile:=real
Reads config/vehicles.yaml, config/field.yaml, config/profile_<profile>.yaml.
Real profile sets auto_initial_arm/offboard=false (agent never self-arms).

Madde 12 (real network architecture): pass vehicle:=<uav_id> to launch only
that ONE vehicle's node group - used to put each UAV in its own ROS_DOMAIN_ID
(export ROS_DOMAIN_ID before this launch call) with a Zenoh bridge carrying
only /swarm/* across domains. Default (vehicle:=-1) launches all vehicles in
one shared domain, i.e. today's single-domain SITL behavior, unchanged.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _load(profile_name):
    share = get_package_share_directory('laplacian_swarm')
    with open(os.path.join(share, 'config', 'vehicles.yaml')) as f:
        veh = yaml.safe_load(f)
    with open(os.path.join(share, 'config', f'profile_{profile_name}.yaml')) as f:
        prof = yaml.safe_load(f)
    with open(os.path.join(share, 'config', 'field.yaml')) as f:
        field = yaml.safe_load(f)
    return veh, prof, field


def _cam_topic(model):
    return f'/world/default/model/{model}/link/camera_link/sensor/fpv_camera/image'


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration('profile').perform(context)
    vehicle_filter = int(LaunchConfiguration('vehicle').perform(context))
    veh, prof, field = _load(profile)
    team_id = int(veh.get('team_id', 1))
    n = len(veh['vehicles'])
    origin = field.get('field_origin') or {'lat_deg': 0.0, 'lon_deg': 0.0, 'alt_m': 0.0}
    # Madde 7: fixed uav_id -> slot_id table from the manifest, identical on
    # every agent (SW-07) - NOT a per-leg dynamic ranking of live positions.
    slot_table = [0] * n
    for v in veh['vehicles']:
        slot_table[int(v['uav_id'])] = int(v.get('slot_id', v['uav_id']))
    actions = []
    for v in veh['vehicles']:
        if vehicle_filter >= 0 and int(v['uav_id']) != vehicle_filter:
            continue
        ns = v['namespace'].lstrip('/')
        did = int(v['uav_id'])
        model = v['gz_model']
        gz_img = _cam_topic(model)
        ros_img = f'/{ns}/down_cam/image'
        actions.append(GroupAction([
            PushRosNamespace(ns),
            Node(package='laplacian_swarm', executable='px4_gateway_node',
                 name='px4_gateway_node', output='screen',
                 parameters=[{'system_id': int(v['system_id'])}]),
            Node(package='laplacian_swarm', executable='localization_node',
                 name='localization_node', output='screen',
                 parameters=[{
                     'field_origin_lat_deg': float(origin['lat_deg']),
                     'field_origin_lon_deg': float(origin['lon_deg']),
                     'field_origin_alt_m': float(origin['alt_m']),
                 }]),
            Node(package='laplacian_swarm', executable='swarm_agent_node',
                 name='swarm_agent_node', output='screen',
                 parameters=[{
                     'drone_id': did,
                     'system_id': int(v['system_id']),
                     'num_uavs': n,
                     'team_id': team_id,
                     'spawn_gz': list(v['spawn_gz']),
                     'slot_table': slot_table,
                     # Madde 11 (SW-24-adjacent): per-ID return altitude, so
                     # multiple UAVs transiting home don't share one altitude.
                     'return_alt': float(v.get('return_altitude_agl_m', 18.0)),
                     'auto_initial_arm': bool(prof['auto_initial_arm']),
                     'auto_initial_offboard': bool(prof['auto_initial_offboard']),
                 }]),
            Node(package='ros_gz_image', executable='image_bridge',
                 name='cam_bridge', output='screen',
                 arguments=[gz_img], remappings=[(gz_img, ros_img)]),
            Node(package='laplacian_swarm', executable='vision_node',
                 name='vision_node', output='screen',
                 parameters=[{'drone_id': did, 'image_topic': ros_img}]),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sitl',
                              description='sitl | real'),
        DeclareLaunchArgument('vehicle', default_value='-1',
                              description='uav_id to launch alone (Madde 12 per-domain '
                                           'mode), or -1 for all vehicles in one domain'),
        OpaqueFunction(function=_launch_setup),
    ])
