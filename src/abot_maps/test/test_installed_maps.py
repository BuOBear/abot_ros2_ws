"""Validate the installed map pairs and their migration provenance."""

import hashlib
import json
import pathlib
import re
import sys
import unittest

import yaml


SHARE_DIR = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[1]
MAPS_DIR = SHARE_DIR / 'maps'
MANIFEST_PATH = MAPS_DIR / 'manifest.json'


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def pgm_dimensions(path):
    """Read the width and height tokens from a P2 or P5 Netpbm header."""
    data = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data):
            if data[index] in b' \t\r\n\v\f':
                index += 1
            elif data[index:index + 1] == b'#':
                newline = data.find(b'\n', index)
                index = len(data) if newline < 0 else newline + 1
            else:
                break
        if index >= len(data):
            raise ValueError(f'Incomplete PGM header: {path}')
        start = index
        while index < len(data) and data[index] not in b' \t\r\n\v\f#':
            index += 1
        tokens.append(data[start:index])
    if tokens[0] not in (b'P2', b'P5'):
        raise ValueError(f'Unsupported PGM magic {tokens[0]!r}: {path}')
    return int(tokens[1]), int(tokens[2])


class InstalledMapsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MANIFEST_PATH.is_file():
            raise AssertionError(f'Installed map manifest is missing: {MANIFEST_PATH}')
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding='utf-8'))

    def test_all_source_images_and_yaml_interpretations_are_installed(self):
        manifest = self.manifest
        self.assertEqual(manifest['package'], 'abot_maps')
        self.assertIsNone(manifest['competition_assignment'])
        self.assertEqual(manifest['map_count'], 2)
        self.assertEqual(len(manifest['maps']), 2)
        self.assertEqual(len(manifest['source_images']), 2)
        self.assertEqual(manifest['candidate_yaml_count'], 0)
        self.assertEqual(manifest['installed_yaml_count'], 2)
        self.assertEqual(
            {entry['source_package'] for entry in manifest['maps']},
            {'robot_slam'},
        )

        installed_yaml_paths = set()
        installed_images = set()
        for image in manifest['source_images']:
            with self.subTest(image=image['installed_image']):
                image_relative = pathlib.Path(image['installed_image'])
                self.assertFalse(image_relative.is_absolute())
                image_path = SHARE_DIR / image_relative
                self.assertTrue(image_path.is_file(), image_path)
                self.assertEqual(sha256(image_path), image['sha256'])
                width, height = pgm_dimensions(image_path)
                self.assertEqual((width, height), (image['width'], image['height']))
                installed_images.add(image_relative)

        for entry in manifest['maps']:
            canonical = entry['canonical']
            with self.subTest(map=canonical['installed_yaml']):
                yaml_relative = pathlib.Path(canonical['installed_yaml'])
                image_relative = pathlib.Path(canonical['installed_image'])
                self.assertFalse(yaml_relative.is_absolute())
                self.assertFalse(image_relative.is_absolute())
                yaml_path = SHARE_DIR / yaml_relative
                image_path = SHARE_DIR / image_relative
                self.assertTrue(yaml_path.is_file(), yaml_path)
                self.assertTrue(image_path.is_file(), image_path)
                self.assertEqual(sha256(image_path), canonical['installed_image_sha256'])
                self.assertEqual(sha256(yaml_path), canonical['installed_yaml_sha256'])
                self.assertRegex(entry['source_stem_image_sha256'], r'^[0-9a-f]{64}$')
                self.assertRegex(entry['source_yaml_sha256'], r'^[0-9a-f]{64}$')
                self.assertTrue(pathlib.Path(entry['source_stem_image']).is_absolute())
                self.assertTrue(pathlib.Path(entry['source_yaml']).is_absolute())

                metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
                self.assertEqual(metadata['image'], image_path.name)
                self.assertEqual(
                    metadata['image'], pathlib.Path(entry['source_yaml_image_value']).name
                )
                self.assertEqual(yaml_path.parent, image_path.parent)
                self.assertGreater(float(metadata['resolution']), 0.0)
                self.assertEqual(len(metadata['origin']), 3)
                self.assertIn(int(metadata['negate']), (0, 1))
                self.assertGreater(float(metadata['occupied_thresh']), float(metadata['free_thresh']))
                self.assertGreaterEqual(float(metadata['free_thresh']), 0.0)
                self.assertLessEqual(float(metadata['occupied_thresh']), 1.0)

                self.assertEqual(
                    (pgm_dimensions(image_path)),
                    (canonical['image_width'], canonical['image_height']),
                )
                installed_yaml_paths.add(yaml_relative)

                for candidate in entry['candidates']:
                    candidate_yaml_relative = pathlib.Path(candidate['installed_yaml'])
                    candidate_image_relative = pathlib.Path(candidate['installed_image'])
                    candidate_yaml_path = SHARE_DIR / candidate_yaml_relative
                    candidate_image_path = SHARE_DIR / candidate_image_relative
                    self.assertTrue(candidate_yaml_path.is_file(), candidate_yaml_path)
                    self.assertTrue(candidate_image_path.is_file(), candidate_image_path)
                    self.assertEqual(candidate['interpretation'], 'stem-matched-candidate (unverified)')
                    self.assertEqual(sha256(candidate_yaml_path), candidate['installed_yaml_sha256'])
                    self.assertEqual(sha256(candidate_image_path), candidate['installed_image_sha256'])
                    candidate_metadata = yaml.safe_load(
                        candidate_yaml_path.read_text(encoding='utf-8')
                    )
                    self.assertEqual(candidate_metadata['image'], candidate_image_path.name)
                    self.assertEqual(candidate_yaml_path.parent, candidate_image_path.parent)
                    self.assertEqual(
                        candidate_image_relative.name,
                        pathlib.Path(entry['source_stem_image']).name,
                    )
                    self.assertEqual(
                        {key: value for key, value in candidate_metadata.items() if key != 'image'},
                        {key: value for key, value in metadata.items() if key != 'image'},
                    )
                    self.assertEqual(
                        pgm_dimensions(candidate_image_path),
                        (candidate['image_width'], candidate['image_height']),
                    )
                    installed_yaml_paths.add(candidate_yaml_relative)

        expected_yaml_paths = {
            pathlib.Path(entry['canonical']['installed_yaml']) for entry in manifest['maps']
        }
        expected_yaml_paths.update(
            pathlib.Path(candidate['installed_yaml'])
            for entry in manifest['maps']
            for candidate in entry['candidates']
        )
        self.assertEqual(installed_yaml_paths, expected_yaml_paths)
        self.assertEqual(installed_images, {
            pathlib.Path(image['installed_image']) for image in manifest['source_images']
        })
        self.assertEqual(
            len(list(MAPS_DIR.rglob('*.yaml'))),
            manifest['installed_yaml_count'],
            'Every installed source-compatible or candidate YAML must be represented in the manifest',
        )
        self.assertEqual(
            len(list(MAPS_DIR.rglob('*.pgm'))),
            len(manifest['source_images']),
            'Every source PGM must be represented in the manifest',
        )

    def test_only_user_selected_maps_are_installed(self):
        self.assertEqual(self.manifest['source_yaml_mismatches'], [])
        self.assertEqual(
            {p.relative_to(MAPS_DIR).as_posix() for p in MAPS_DIR.rglob('*') if p.is_file()},
            {'manifest.json', 'robot_slam/1.pgm', 'robot_slam/1.yaml',
             'robot_slam/shoot.pgm', 'robot_slam/shoot.yaml'},
        )


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
