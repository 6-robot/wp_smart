"""Check scan directions through the actual robot model's fixed transforms."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


def test_rear_origin_scan_keeps_obstacles_in_correct_body_directions():
    root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((root / 'smart_bringup/config/smart_base.yaml').read_text())
    params = params['smart_base_node']['ros__parameters']
    model = xacro.process_file(str(root / 'smart_description/urdf/smart.urdf.xacro'))
    robot = ET.fromstring(model.toxml())
    parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    frame = params['laser_frame']
    yaw = 0.0
    while frame != 'base_link':
        joint = parents[frame]
        angles = [float(value) for value in joint.find('origin').get('rpy', '0 0 0').split()]
        assert angles[:2] == [0.0, 0.0]
        yaw += angles[2]
        frame = joint.find('parent').get('link')

    # The requested numeric scan angles must transform into real body directions.
    for angle, expected in ((0, (-1, 0)), (90, (0, -1)),
                            (180, (1, 0)), (270, (0, 1))):
        body_angle = math.radians(angle) + yaw
        assert (math.cos(body_angle), math.sin(body_angle)) == pytest.approx(expected)

    # Preserve the shared model's forward-facing physical laser frame for Wi-Fi.
    laser_origin = parents['laser_link'].find('origin')
    assert laser_origin.get('rpy') == '0 0 0'

    # Every raw point keeps its former body direction after the 180-degree reindexing.
    for raw in range(0, 360, 5):
        scan_angle = params['scan_angle_offset_deg'] + params['scan_angle_direction'] * raw
        body_angle = math.radians(scan_angle) + yaw
        previous_angle = math.radians(180 - raw)
        assert (math.cos(body_angle), math.sin(body_angle)) == pytest.approx(
            (math.cos(previous_angle), math.sin(previous_angle)))
