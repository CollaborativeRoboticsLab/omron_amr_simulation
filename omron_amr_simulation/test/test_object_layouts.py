import collections
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('preview', PACKAGE / 'launch' / 'simulation.launch.py')
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)
DRIVE_SPEC = importlib.util.spec_from_file_location('base_drive', PACKAGE / 'launch' / 'base_drive.py')
base_drive = importlib.util.module_from_spec(DRIVE_SPEC)
DRIVE_SPEC.loader.exec_module(base_drive)


def test_drive_geometry():
    root = ET.fromstring('<robot name="test"><link name="base_link"/><link name="main_body"><collision/></link></robot>')
    base_drive.add_drive_geometry(root)
    assert len(root.findall("link[@name='main_body']/collision")) == 3
    for side in ('left', 'right'):
        joint = root.find(f"joint[@name='sim_{side}_wheel_joint']")
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert joint.find('axis').get('xyz') == '0 1 0'
        assert root.find(f"link[@name='sim_{side}_wheel']/collision/geometry/cylinder").get('radius') == '0.1'


def test_drive_geometry_with_prefixed_body():
    root = ET.fromstring('<robot name="test"><link name="base_link"/><link name="ld250_main_body"/></robot>')
    base_drive.add_drive_geometry(root, body_name='ld250_main_body')
    assert len(root.findall("link[@name='ld250_main_body']/collision")) == 3


def test_lidar_configuration():
    root = ET.fromstring('<robot name="test"><link name="base_link"/></robot>')
    base_drive.add_lidars(root, prefixed_frames=True)
    for topic, frame, count in (('scan', 'laser_frame', '481'), ('scan_low', 'laser_frame_low', '253')):
        sensor = root.find(f"gazebo/sensor[@name='sim_{topic}']")
        assert sensor.findtext('topic') == f'/sim_lidar/{topic}_raw'
        assert sensor.findtext('gz_frame_id') == frame
        assert sensor.findtext('lidar/scan/horizontal/samples') == count
        assert root.find(f"joint[@name='sim_{frame}_alias']/parent").get('link') == f'ld250_{frame}'
    with (PACKAGE / 'config' / 'lidar_filter.yaml').open() as config_file:
        config = yaml.safe_load(config_file)['/**']['ros__parameters']['filter1']
    assert config['type'] == 'laser_filters/LaserScanBoxFilter'
    assert config['params']['box_frame'] == 'base_link'
    assert config['params']['invert'] is False


def test_ground_truth_configuration():
    root = ET.fromstring('<robot name="test"/>')
    base_drive.add_ground_truth(root)
    plugin = root.find("gazebo/plugin[@name='gz::sim::systems::PosePublisher']")
    assert plugin.findtext('publish_model_pose') == 'true'
    assert plugin.findtext('publish_link_pose') == 'false'
    assert plugin.findtext('use_pose_vector_msg') == 'false'


def test_contact_sensors_do_not_require_rendering_engine():
    world = ET.fromstring('<world name="test"/>')
    plugin = preview.add_sensors_system(world)
    assert plugin.get('filename') == 'gz-sim-sensors-system'
    assert plugin.find('render_engine') is None


def test_dock_geometry():
    world = ET.fromstring('<world name="test"/>')
    base_drive.add_dock(world, 1.0, 2.0, 0.0)
    dock = world.find("model[@name='sim_charging_dock']")
    assert dock.findtext('static') == 'true'
    assert [float(value) for value in dock.findtext('pose').split()][:2] == pytest.approx([0.45, 2.0])
    assert dock.find('link/collision/geometry/box') is not None
    assert len(base_drive.docking_nodes('ld250', 1.0, 2.0, 0.0)) == 2


def included_models(root, prefix):
    return [item for item in root.find('world').findall('include')
            if item.findtext('uri', '').startswith(prefix)]


def layout(objects):
    return sorted((item.findtext('name'), item.findtext('pose')) for item in objects)


def test_grasp_test_layout():
    world_path = PACKAGE / 'worlds' / 'grasp_test.sdf'
    roots = [ET.parse(world_path).getroot() for _ in range(3)]
    preview.randomize_grasp_objects(roots[1], '42')
    preview.randomize_grasp_objects(roots[2], '42')

    original = included_models(roots[0], 'model://manipulation_')
    shuffled = included_models(roots[1], 'model://manipulation_')
    assert len(original) == len(shuffled) == 25
    assert layout(shuffled) == layout(included_models(roots[2], 'model://manipulation_'))
    assert layout(original) != layout(shuffled)

    expected_types = {f'model://manipulation_{name}'
                      for name in ('box', 'cylinder', 'sphere', 'block', 'bottle')}
    for table_y in (-4, -2, 0, 2, 4):
        on_table = [item for item in shuffled
                    if abs(float(item.findtext('pose').split()[1]) - table_y) < 0.5]
        assert len(on_table) == 5
        assert {item.findtext('uri') for item in on_table} == expected_types
        assert len({item.findtext('pose') for item in on_table}) == 5


def test_warehouse_box_layout():
    world_path = PACKAGE / 'worlds' / 'warehouse.sdf'
    roots = [ET.parse(world_path).getroot() for _ in range(3)]
    preview.randomize_warehouse_boxes(roots[1], '42')
    preview.randomize_warehouse_boxes(roots[2], '42')

    fixed = included_models(roots[0], 'model://warehouse_box_')
    shuffled = included_models(roots[1], 'model://warehouse_box_')
    assert len(fixed) == len(shuffled) == 20
    assert layout(shuffled) == layout(included_models(roots[2], 'model://warehouse_box_'))
    assert layout(fixed) != layout(shuffled)

    expected_colors = {f'model://warehouse_box_{color}': 4
                       for color in ('blue', 'red', 'green', 'black', 'brown')}
    for objects in (fixed, shuffled):
        assert collections.Counter(item.findtext('uri') for item in objects) == expected_colors
        assert len({item.findtext('pose') for item in objects}) == 20
        racks = collections.Counter(min((-3, 0, 3), key=lambda rack: abs(
            float(item.findtext('pose').split()[1]) - rack)) for item in objects)
        assert racks == {-3: 7, 0: 7, 3: 6}