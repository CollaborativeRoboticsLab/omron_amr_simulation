import collections
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('preview', PACKAGE / 'launch' / 'preview.launch.py')
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


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