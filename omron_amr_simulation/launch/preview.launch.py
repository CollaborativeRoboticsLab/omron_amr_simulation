import copy
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def create_preview(context):
    model = LaunchConfiguration('model').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    worlds = {
        'empty': (os.path.join(get_package_share_directory('omron_amr_simulation'), 'worlds', 'empty.sdf'), 'empty'),
        'warehouse': (os.path.join(get_package_share_directory('omron_amr_simulation'), 'worlds', 'warehouse.sdf'), 'industrial-warehouse'),
    }
    if world_name not in worlds:
        raise ValueError(f'Unsupported world: {world_name}; choose {", ".join(worlds)}')
    world, gazebo_world = worlds[world_name]
    descriptions = {
        'ld90': ('omron_amr_simulation', 'urdf/ld90.urdf.xacro'),
        'ld250': ('omron_amr_simulation', 'urdf/ld250.urdf.xacro'),
        'ld250_tm12x': ('moma_description', 'xacro/ld250_tm12x.urdf.xacro'),
    }
    if model not in descriptions:
        raise ValueError(f'Unsupported model: {model}; choose {", ".join(descriptions)}')

    package, relative_path = descriptions[model]
    description_package = LaunchConfiguration('description_package').perform(context)
    description_file = LaunchConfiguration('description_file').perform(context)
    if bool(description_package) != bool(description_file):
        raise ValueError('description_package and description_file must be specified together')
    if description_package:
        package, relative_path = description_package, description_file
    description_path = os.path.join(get_package_share_directory(package), relative_path)
    root = ET.fromstring(xacro.process_file(description_path).toxml())
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('filename') == 'libgazebo_ros_control.so':
                gazebo.remove(plugin)
    ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
    description = ET.tostring(root, encoding='unicode')
    gazebo_root = copy.deepcopy(root)
    for visual in gazebo_root.findall('.//visual'):
        mesh = visual.find('geometry/mesh')
        material = visual.find('material')
        if mesh is not None and mesh.get('filename', '').lower().endswith('.obj') and material is not None:
            visual.remove(material)
    gazebo_description = ET.tostring(gazebo_root, encoding='unicode')

    resource_packages = set()
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            resource_packages.add(filename[len('package://'):].split('/', 1)[0])
    resource_paths = [os.path.dirname(get_package_share_directory(name)) for name in sorted(resource_packages)]
    resource_paths.append(os.environ.get('GZ_SIM_RESOURCE_PATH', ''))
    gazebo_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'gz_args': f'-r -s {world}', 'on_exit_shutdown': 'true'}.items(),
        ),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', gazebo_world, '-name', model, '-string', gazebo_description,
                     '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                     '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='ld250', description='ld90, ld250, or ld250_tm12x'),
        DeclareLaunchArgument('world', default_value='empty', description='empty or warehouse'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position in meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo graphical client'),
        DeclareLaunchArgument('description_package', default_value='', description='Package containing an alternate robot Xacro'),
        DeclareLaunchArgument('description_file', default_value='', description='Xacro path relative to the description package'),
        OpaqueFunction(function=create_preview),
    ])