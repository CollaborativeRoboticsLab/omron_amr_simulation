import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def add_lidars(root, prefixed_frames=False):
    sensors = (
        ('scan', 'laser_frame', '0.45 0 0.195', 481, -2.09439510239, 2.09439510239, 40.0),
        ('scan_low', 'laser_frame_low', '0.40 0 0.060', 253, -1.09955742876, 1.09955742876, 4.0),
    )
    gazebo = ET.SubElement(root, 'gazebo', {'reference': 'base_link'})
    for topic, frame, position, samples, angle_min, angle_max, maximum in sensors:
        if prefixed_frames:
            ET.SubElement(root, 'link', {'name': frame})
            joint = ET.SubElement(root, 'joint', {'name': f'sim_{frame}_alias', 'type': 'fixed'})
            ET.SubElement(joint, 'parent', {'link': f'ld250_{frame}'})
            ET.SubElement(joint, 'child', {'link': frame})
        sensor = ET.SubElement(gazebo, 'sensor', {'name': f'sim_{topic}', 'type': 'gpu_lidar'})
        ET.SubElement(sensor, 'pose').text = f'{position} 0 0 0'
        ET.SubElement(sensor, 'topic').text = f'/sim_lidar/{topic}_raw'
        ET.SubElement(sensor, 'gz_frame_id').text = frame
        ET.SubElement(sensor, 'update_rate').text = '5'
        lidar = ET.SubElement(sensor, 'lidar')
        horizontal = ET.SubElement(ET.SubElement(lidar, 'scan'), 'horizontal')
        for name, value in (('samples', samples), ('resolution', 1), ('min_angle', angle_min), ('max_angle', angle_max)):
            ET.SubElement(horizontal, name).text = str(value)
        scan_range = ET.SubElement(lidar, 'range')
        for name, value in (('min', 0.02), ('max', maximum), ('resolution', 0.01)):
            ET.SubElement(scan_range, name).text = str(value)


def lidar_nodes():
    config = os.path.join(get_package_share_directory('omron_amr_simulation'), 'config', 'lidar_filter.yaml')
    nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
        '/sim_lidar/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/sim_lidar/scan_low_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ], output='screen')]
    for topic in ('scan', 'scan_low'):
        nodes.append(Node(package='laser_filters', executable='scan_to_scan_filter_chain',
                          name=f'sim_{topic}_filter', parameters=[config, {'use_sim_time': True}],
                          remappings=[('scan', f'/sim_lidar/{topic}_raw'), ('scan_filtered', f'/{topic}')],
                          output='screen'))
    return nodes


def add_ground_truth(root):
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
        'filename': 'gz-sim-pose-publisher-system',
        'name': 'gz::sim::systems::PosePublisher',
    })
    for name, value in (('publish_model_pose', 'true'), ('publish_link_pose', 'false'),
                        ('use_pose_vector_msg', 'false'), ('update_frequency', '20')):
        ET.SubElement(plugin, name).text = value


def add_dock(world, x, y, yaw):
    model = ET.SubElement(world, 'model', {'name': 'sim_charging_dock'})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = (
        f'{x - 0.55 * math.cos(yaw)} {y - 0.55 * math.sin(yaw)} 0 0 0 {yaw}')
    link = ET.SubElement(model, 'link', {'name': 'dock_link'})
    for kind in ('collision', 'visual'):
        element = ET.SubElement(link, kind, {'name': f'dock_{kind}'})
        ET.SubElement(element, 'pose').text = '0 0 0.08 0 0 0'
        box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = '0.2 0.6 0.16'
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            ET.SubElement(material, 'ambient').text = '0.18 0.2 0.22 1'
            ET.SubElement(material, 'diffuse').text = '0.25 0.28 0.3 1'


def docking_nodes(model, x, y, yaw):
    topic = f'/model/{model}/pose'
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[f'{topic}@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
             remappings=[(topic, '/sim_base/ground_truth')], output='screen'),
        Node(package='omron_amr_simulation', executable='sim_docking.py',
             parameters=[{'dock_x': x, 'dock_y': y, 'dock_yaw': yaw}], output='screen'),
    ]


def add_drive_geometry(root, body_name='main_body'):
    body = root.find(f"link[@name='{body_name}']")
    if body is None or root.find("link[@name='base_link']") is None:
        raise ValueError(f'Drive simulation requires base_link and {body_name}')

    for collision in body.findall('collision'):
        body.remove(collision)
    collision = ET.SubElement(body, 'collision')
    ET.SubElement(collision, 'origin', {'xyz': '0 0 0.25'})
    ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', {'size': '0.85 0.60 0.30'})

    for side, offset in (('left', 0.29), ('right', -0.29)):
        link = ET.SubElement(root, 'link', {'name': f'sim_{side}_wheel'})
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', {'value': '0.5'})
        ET.SubElement(inertial, 'inertia', {
            'ixx': '0.0014', 'ixy': '0', 'ixz': '0',
            'iyy': '0.0014', 'iyz': '0', 'izz': '0.0025',
        })
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', {'rpy': f'{math.pi / 2} 0 0'})
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'cylinder', {'radius': '0.1', 'length': '0.06'})
        joint = ET.SubElement(root, 'joint', {'name': f'sim_{side}_wheel_joint', 'type': 'continuous'})
        ET.SubElement(joint, 'parent', {'link': 'base_link'})
        ET.SubElement(joint, 'child', {'link': f'sim_{side}_wheel'})
        ET.SubElement(joint, 'origin', {'xyz': f'0 {offset} 0.1'})
        ET.SubElement(joint, 'axis', {'xyz': '0 1 0'})

    for position in (-0.32, 0.32):
        collision = ET.SubElement(body, 'collision')
        ET.SubElement(collision, 'origin', {'xyz': f'{position} 0 0.04'})
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'sphere', {'radius': '0.04'})

    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
        'filename': 'gz-sim-diff-drive-system',
        'name': 'gz::sim::systems::DiffDrive',
    })
    for name, value in (
            ('left_joint', 'sim_left_wheel_joint'), ('right_joint', 'sim_right_wheel_joint'),
            ('wheel_separation', '0.58'), ('wheel_radius', '0.1'),
            ('topic', '/sim_base/cmd_vel'), ('odom_topic', '/odom'),
            ('tf_topic', '/sim_base/tf'), ('frame_id', 'odom'), ('child_frame_id', 'base_link'),
            ('odom_publish_frequency', '20')):
        ET.SubElement(plugin, name).text = value
    add_ground_truth(root)


def drive_nodes():
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/sim_base/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/sim_base/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ], remappings=[('/sim_base/tf', '/tf')], output='screen'),
        Node(package='omron_amr_simulation', executable='sim_base_command.py',
             name='sim_base_command', output='screen'),
    ]