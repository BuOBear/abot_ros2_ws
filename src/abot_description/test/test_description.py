"""Check the expanded description's TF tree, geometry and optical axes."""

import math
import pathlib
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from collections import Counter


PACKAGE = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'urdf' / 'abot.urdf.xacro'


def rotate(rpy, vector):
    roll, pitch, yaw = rpy
    x, y, z = vector
    y, z = math.cos(roll) * y - math.sin(roll) * z, math.sin(roll) * y + math.cos(roll) * z
    x, z = math.cos(pitch) * x + math.sin(pitch) * z, -math.sin(pitch) * x + math.cos(pitch) * z
    x, y = math.cos(yaw) * x - math.sin(yaw) * y, math.sin(yaw) * x + math.cos(yaw) * y
    return x, y, z


class DescriptionStructureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        expanded = subprocess.run(
            ['xacro', str(MODEL)], check=True, text=True, capture_output=True
        ).stdout
        cls.robot = ET.fromstring(expanded)
        cls.links = {link.attrib['name']: link for link in cls.robot.findall('link')}
        cls.joints = {joint.attrib['name']: joint for joint in cls.robot.findall('joint')}

    def test_tree_is_connected_and_uniquely_parented(self):
        self.assertEqual(len(self.links), len(self.robot.findall('link')))
        self.assertEqual(len(self.joints), len(self.robot.findall('joint')))
        self.assertEqual(len(self.joints), len(self.links) - 1)
        parents = {joint.find('child').attrib['link']: joint.find('parent').attrib['link']
                   for joint in self.joints.values()}
        self.assertEqual(Counter(joint.find('child').attrib['link'] for joint in self.joints.values()),
                         Counter(parents.keys()))
        self.assertEqual(set(self.links) - set(parents), {'base_footprint'})
        self.assertEqual(parents['base_link'], 'base_footprint')
        self.assertEqual(parents['imu_link'], 'base_link')
        self.assertEqual(parents['laser_link'], 'base_link')
        self.assertEqual(parents['camera_link'], 'base_link')
        self.assertEqual(parents['camera_optical_frame'], 'camera_link')
        self.assertEqual(parents['link_left_w'], 'base_link')
        self.assertEqual(parents['link_left_s'], 'base_link')
        self.assertEqual(parents['link_right_w'], 'base_link')
        self.assertEqual(parents['link_right_s'], 'base_link')
        self.assertNotIn('base_imu', self.links)
        for child in parents:
            seen = set()
            current = child
            while current != 'base_footprint':
                self.assertNotIn(current, seen)
                seen.add(current)
                current = parents[current]

    def test_names_and_joint_origins_are_valid(self):
        for name in (*self.links, *self.joints):
            self.assertFalse(name.startswith('/'), name)
        for joint in self.joints.values():
            self.assertIn(joint.find('parent').attrib['link'], self.links)
            self.assertIn(joint.find('child').attrib['link'], self.links)
            origin = joint.find('origin')
            self.assertIsNotNone(origin)
            for key in ('xyz', 'rpy'):
                values = [float(v) for v in origin.attrib[key].split()]
                self.assertEqual(len(values), 3)
                self.assertTrue(all(math.isfinite(v) for v in values))
        for suffix in ('left_w', 'left_s', 'right_w', 'right_s'):
            joint = self.joints[f'joint_{suffix}']
            self.assertEqual(joint.attrib['type'], 'continuous')
            self.assertEqual(joint.find('axis').attrib['xyz'], '0 1 0')

    def test_meshes_resolve_and_have_meter_scale(self):
        seen = set()
        prefix = 'package://abot_description/'
        for mesh in self.robot.iter('mesh'):
            filename = mesh.attrib['filename']
            self.assertTrue(filename.startswith(prefix), filename)
            path = PACKAGE / filename[len(prefix):]
            self.assertTrue(path.is_file(), filename)
            self.assertGreater(path.stat().st_size, 100)
            seen.add(path.name)
            self.assertNotIn('scale', mesh.attrib)
        self.assertEqual(seen, {
            'base_link.STL', 'camera_link.STL', 'link_left_w.STL',
            'link_left_s.STL', 'link_right_w.STL', 'link_right_s.STL'
        })

    def test_optical_frame_rotation(self):
        optical = self.joints['camera_optical_joint']
        self.assertEqual(optical.attrib['type'], 'fixed')
        rpy = tuple(float(v) for v in optical.find('origin').attrib['rpy'].split())
        # Optical z points along camera_link x; optical x points camera_link -y;
        # optical y points camera_link -z (REP 103).
        for actual, expected in (
            (rotate(rpy, (0, 0, 1)), (1, 0, 0)),
            (rotate(rpy, (1, 0, 0)), (0, -1, 0)),
            (rotate(rpy, (0, 1, 0)), (0, 0, -1)),
        ):
            for a, e in zip(actual, expected):
                self.assertAlmostEqual(a, e, places=8)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
