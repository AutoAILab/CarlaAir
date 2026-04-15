import unittest
from unittest.mock import MagicMock, patch
import os
import shutil
import numpy as np
import json
import sys

# Add the directory to sys.path to import the script
sys.path.append(os.path.join(os.getcwd(), 'dataset_generation'))
from agcarla_datagen import AGCarlaGenerator

class TestAGCarlaGenerator(unittest.TestCase):
    def setUp(self):
        # Create a mock args object
        self.args = MagicMock()
        self.args.host = '127.0.0.1'
        self.args.port = 2000
        self.args.airsim_port = 41451
        self.args.tm_port = 8000
        self.args.width = 1280
        self.args.height = 720
        self.args.mode = 'record'
        self.args.out = 'mock_test_output'
        self.args.num_frames = 2

        # Patch the Client connections
        self.patch_carla = patch('carla.Client')
        self.patch_airsim = patch('airsim.MultirotorClient')
        
        self.mock_carla = self.patch_carla.start()
        self.mock_airsim = self.patch_airsim.start()
        
        # Setup mock world/map
        self.mock_world = MagicMock()
        self.mock_carla.return_value.get_world.return_value = self.mock_world
        self.mock_world.get_blueprint_library.return_value = MagicMock()
        self.mock_world.get_map.return_value = MagicMock()
        self.mock_world.get_settings.return_value = MagicMock()
        
        # Setup mock airsim listVehicles
        self.mock_airsim.return_value.listVehicles.return_value = ['UAV_1']
        
        # Setup mock weather default
        mock_weather = MagicMock()
        mock_weather.cloudiness = 0
        mock_weather.precipitation = 0
        mock_weather.fog_density = 0
        self.mock_world.get_weather.return_value = mock_weather
        
        # Setup mock snapshot default
        mock_snapshot = MagicMock()
        mock_snapshot.timestamp.elapsed_seconds = 0.0
        self.mock_world.get_snapshot.return_value = mock_snapshot

    def tearDown(self):
        self.patch_carla.stop()
        self.patch_airsim.stop()
        if os.path.exists('mock_test_output'):
            shutil.rmtree('mock_test_output')

    def test_init_and_directories(self):
        gen = AGCarlaGenerator(self.args)
        # Check if directories were created
        for sub in ['images', 'depth', 'seg', 'gbuffer', 'metadata']:
            self.assertTrue(os.path.exists(os.path.join(gen.output_dir, sub)))

    def test_ray_map_math(self):
        gen = AGCarlaGenerator(self.args)
        ray_map = gen.get_ray_map()
        self.assertEqual(ray_map.shape, (720, 1280, 3))
        # Center ray should be normalized and pointing forward (near [0, 0, 1])
        center_ray = ray_map[360, 640]
        self.assertAlmostEqual(np.linalg.norm(center_ray), 1.0, places=5)
        self.assertTrue(center_ray[2] > 0.8) # Forward-looking

    def test_metadata_serialization(self):
        gen = AGCarlaGenerator(self.args)
        mock_data = {'test': 123}
        gen.save_metadata(0, mock_data)
        meta_file = os.path.join(gen.output_dir, 'metadata', '000000.json')
        self.assertTrue(os.path.exists(meta_file))
        with open(meta_file, 'r') as f:
            data = json.load(f)
        self.assertEqual(data['test'], 123)

    def test_openlabel_tagging(self):
        gen = AGCarlaGenerator(self.args)
        # Mock map name
        gen.map.name = '/Game/Carla/Maps/Town10HD'
        # Mock weather
        mock_weather = MagicMock()
        mock_weather.cloudiness = 10
        mock_weather.precipitation = 0
        mock_weather.fog_density = 0
        self.mock_world.get_weather.return_value = mock_weather
        
        tags = gen.get_odd_tags()
        self.assertIn('Town_10HD', tags)
        self.assertIn('SeqBaseline', tags)
        self.assertIn('WeatherClear', tags)
        self.assertIn('UAV_1', tags)

    def test_openlabel_manifest_append(self):
        gen = AGCarlaGenerator(self.args)
        # Mock world snapshot for timestamp
        self.mock_world.get_snapshot.return_value.timestamp.elapsed_seconds = 1234.56
        
        gen.append_openlabel_frame(0)
        
        manifest_file = os.path.join(gen.output_dir, 'master_manifest.jsonl')
        self.assertTrue(os.path.exists(manifest_file))
        
        with open(manifest_file, 'r') as f:
            line = f.readline()
            data = json.loads(line)
            
        self.assertEqual(data['openlabel']['metadata']['schema_version'], '1.0.0')
        self.assertIn('0', data['openlabel']['frames'])
        frame_props = data['openlabel']['frames']['0']['frame_properties']
        self.assertEqual(frame_props['timestamp'], 1234.56)
        
        # Check external resources
        resources = frame_props['external_resources']
        resource_names = [r['name'] for r in resources]
        self.assertIn('ugv_rgb', resource_names)
        self.assertIn('UAV_1_rgb', resource_names)

if __name__ == '__main__':
    unittest.main()
