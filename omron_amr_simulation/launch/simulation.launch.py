import copy
import importlib.util
import math
import os
import random
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import evaluate_condition_expression, IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def randomize_warehouse_boxes(root, seed=None):
    world = root.find('world')
    for include in world.findall('include'):
        if include.findtext('name', '').startswith('warehouse_box_'):
            world.remove(include)

    generator = random.Random(seed)
    slots = []
    for rack_y, count in ((-3.0, 7), (0.0, 7), (3.0, 6)):
        rack_slots = [(-2.7, rack_y + offset, height)
                      for height in (0.44, 0.79, 1.14, 1.49)
                      for offset in (-0.8, -0.4, 0.0, 0.4, 0.8)]
        slots.extend(generator.sample(rack_slots, count))
    generator.shuffle(slots)
    colors = [color for color in ('blue', 'red', 'green', 'black', 'brown') for _ in range(4)]
    generator.shuffle(colors)
    counts = {color: 0 for color in colors}
    for color, (x, y, z) in zip(colors, slots):
        counts[color] += 1
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'name').text = f'warehouse_box_{color}_{counts[color]}'
        ET.SubElement(include, 'uri').text = f'model://warehouse_box_{color}'
        ET.SubElement(include, 'pose').text = f'{x} {y} {z} 0 0 0'


def randomize_grasp_objects(root, seed=None):
    world = root.find('world')
    includes = world.findall('include')
    table_positions = [float(include.findtext('pose').split()[1]) for include in includes
                       if include.findtext('uri') == 'model://grasp_worktable']
    groups = {position: [] for position in table_positions}
    for include in includes:
        if include.findtext('uri', '').startswith('model://manipulation_'):
            object_y = float(include.findtext('pose').split()[1])
            nearest_table = min(table_positions, key=lambda position: abs(position - object_y))
            groups[nearest_table].append(include)

    generator = random.Random(seed)
    for objects in groups.values():
        poses = [item.findtext('pose') for item in objects]
        generator.shuffle(poses)
        for item, pose in zip(objects, poses):
            item.find('pose').text = pose


def create_preview(context):
    model = LaunchConfiguration('model').perform(context)
    world_only = evaluate_condition_expression(context, [LaunchConfiguration('world_only')])
    drive = evaluate_condition_expression(context, [LaunchConfiguration('drive')])
    lidar = evaluate_condition_expression(context, [LaunchConfiguration('lidar')])
    render_sensors = evaluate_condition_expression(context, [LaunchConfiguration('render_sensors')])
    docking = evaluate_condition_expression(context, [LaunchConfiguration('docking')])
    if docking and not world_only and not drive:
        raise ValueError('Docking simulation requires drive:=true')
    if docking:
        dock_x, dock_y, dock_yaw = (float(LaunchConfiguration(name).perform(context))
                                     for name in ('dock_x', 'dock_y', 'dock_yaw'))
        if not all(math.isfinite(value) for value in (dock_x, dock_y, dock_yaw)):
            raise ValueError('Dock coordinates must be finite')
    if drive or lidar or docking:
        spec = importlib.util.spec_from_file_location('base_drive', os.path.join(
            get_package_share_directory('omron_amr_simulation'), 'launch', 'base_drive.py'))
        base_drive = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(base_drive)
    world_name = LaunchConfiguration('world').perform(context)
    worlds = {
        'empty': (os.path.join(get_package_share_directory('omron_amr_simulation'), 'worlds', 'empty.sdf'), 'empty'),
        'grasp_test': (os.path.join(get_package_share_directory('omron_amr_simulation'), 'worlds', 'grasp_test.sdf'), 'grasp_test'),
        'warehouse': (os.path.join(get_package_share_directory('omron_amr_simulation'), 'worlds', 'warehouse.sdf'), 'industrial-warehouse'),
    }
    if world_name not in worlds:
        raise ValueError(f'Unsupported world: {world_name}; choose {", ".join(worlds)}')
    world, gazebo_world = worlds[world_name]
    resource_packages = set()
    if not world_only:
        descriptions = {
            'ld90': ('omron_amr_simulation', 'urdf/ld90.urdf.xacro'),
            'ld250': ('omron_amr_simulation', 'urdf/ld250.urdf.xacro'),
        }
        if model not in descriptions:
            raise ValueError(f'Unsupported model: {model}; choose {", ".join(descriptions)}')
        package, relative_path = descriptions[model]
        description_path = os.path.join(get_package_share_directory(package), relative_path)
        root = ET.fromstring(xacro.process_file(description_path).toxml())
        for gazebo in root.findall('gazebo'):
            for plugin in gazebo.findall('plugin'):
                if plugin.get('filename') == 'libgazebo_ros_control.so':
                    gazebo.remove(plugin)
        if drive or lidar:
            if model != 'ld250':
                raise ValueError('Drive and lidar simulation currently support only ld250')
        if drive:
            base_drive.add_drive_geometry(root)
        else:
            ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
        if lidar:
            base_drive.add_lidars(root)
        description = ET.tostring(root, encoding='unicode')
        gazebo_root = copy.deepcopy(root)
        for visual in gazebo_root.findall('.//visual'):
            mesh = visual.find('geometry/mesh')
            material = visual.find('material')
            if mesh is not None and mesh.get('filename', '').lower().endswith('.obj') and material is not None:
                visual.remove(material)
        gazebo_description = ET.tostring(gazebo_root, encoding='unicode')
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename', '')
            if filename.startswith('package://'):
                resource_packages.add(filename[len('package://'):].split('/', 1)[0])
    resource_paths = [os.path.dirname(get_package_share_directory(name)) for name in sorted(resource_packages)]
    resource_paths.append(os.path.join(get_package_share_directory('omron_amr_simulation'), 'models'))
    resource_paths.append(context.environment.get('GZ_SIM_RESOURCE_PATH', ''))
    gazebo_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    cleanup = []
    randomizers = {
        'warehouse': ('randomize_boxes', 'box_seed', randomize_warehouse_boxes),
        'grasp_test': ('randomize_objects', 'object_seed', randomize_grasp_objects),
    }
    randomize = world_name in randomizers and evaluate_condition_expression(context, [LaunchConfiguration(randomizers[world_name][0])])
    if randomize or lidar or render_sensors or docking:
        tree = ET.parse(world)
        if docking:
            base_drive.add_dock(tree.getroot().find('world'), dock_x, dock_y, dock_yaw)
        if lidar or render_sensors:
            plugin = ET.SubElement(tree.getroot().find('world'), 'plugin', {
                'filename': 'gz-sim-sensors-system', 'name': 'gz::sim::systems::Sensors'})
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
        if randomize:
            _, seed_argument, randomizer = randomizers[world_name]
            seed = LaunchConfiguration(seed_argument).perform(context)
            randomizer(tree.getroot(), seed or None)
        temporary_world = tempfile.TemporaryDirectory(prefix=f'omron_{world_name}_')
        world = os.path.join(temporary_world.name, f'{world_name}.sdf')
        tree.write(world, encoding='utf-8', xml_declaration=True)
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context, directory=temporary_world: directory.cleanup())
        ])))

    nodes = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'gz_args': f'-r -s {world}', 'on_exit_shutdown': 'true'}.items(),
        ),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
    ]
    if not world_only:
        nodes.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
            Node(package='joint_state_publisher', executable='joint_state_publisher',
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', gazebo_world, '-name', model, '-string', gazebo_description,
                            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                            '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')], output='screen'),
        ])
        if drive:
            nodes.extend(base_drive.drive_nodes())
        if lidar:
            nodes.extend(base_drive.lidar_nodes())
        if docking:
            nodes.extend(base_drive.docking_nodes(model, dock_x, dock_y, dock_yaw))
    return cleanup + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='ld250', description='ld90 or ld250'),
        DeclareLaunchArgument('world_only', default_value='false', description='Start the world without a robot'),
        DeclareLaunchArgument('drive', default_value='false', description='Enable experimental LD250 drive simulation'),
        DeclareLaunchArgument('lidar', default_value='false', description='Enable experimental LD250 laser scans'),
        DeclareLaunchArgument('render_sensors', default_value='false', description='Enable Gazebo rendering sensors in the world'),
        DeclareLaunchArgument('docking', default_value='false', description='Enable the experimental simulated dock'),
        DeclareLaunchArgument('dock_x', default_value='0.0', description='Docked robot X position in world coordinates'),
        DeclareLaunchArgument('dock_y', default_value='0.0', description='Docked robot Y position in world coordinates'),
        DeclareLaunchArgument('dock_yaw', default_value='0.0', description='Docked robot yaw in radians'),
        DeclareLaunchArgument('world', default_value='empty', description='empty, grasp_test, or warehouse'),
        DeclareLaunchArgument('randomize_boxes', default_value='false', description='Randomize warehouse box positions'),
        DeclareLaunchArgument('box_seed', default_value='', description='Optional seed for warehouse box positions'),
        DeclareLaunchArgument('randomize_objects', default_value='false', description='Randomize grasp-test object positions'),
        DeclareLaunchArgument('object_seed', default_value='', description='Optional seed for grasp-test object positions'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position in meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo graphical client'),
        OpaqueFunction(function=create_preview),
    ])