#!/usr/bin/env python3
"""Replace the generated VRX sensor with a pinned Garden MID-360 approximation."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_world import element, set_text

ROOT = Path(__file__).resolve().parents[1]


def prepare(generated):
    generated = Path(generated)
    path = generated/'usv_nav.sdf'
    tree = ET.parse(path)
    boat = tree.find(".//model[@name='wamv']")
    sensor = boat.find(".//sensor[@name='lidar_wamv_sensor']")
    if sensor is None:
        raise ValueError('Expected generated VRX lidar to replace')
    sensor.set('name', 'mid360')
    sensor.set('type', 'gpu_lidar')
    set_text(sensor, 'topic', '/mid360/dense_scan')
    set_text(sensor, 'update_rate', 50)
    set_text(sensor, 'always_on', 'true')
    ray = sensor.find('ray')
    ray.tag = 'lidar'
    for axis, count, lo, hi in [('horizontal', 901, -180., 180.),
                                ('vertical', 151, -7.4, 52.4)]:
        scan = ray.find('scan/'+axis)
        set_text(scan, 'samples', count)
        set_text(scan, 'min_angle', math.radians(lo))
        set_text(scan, 'max_angle', math.radians(hi))
    set_text(ray.find('range'), 'min', .1)
    set_text(ray.find('range'), 'max', 40.)
    for noise in list(ray.findall('noise')):
        ray.remove(noise)
    pattern = ROOT/'external/livox_gz_sim/scan_patterns/mid360.csv'
    plugin = element(sensor, 'plugin', name='livox_gz_sim::LivoxLidar', filename='liblivox_gz_sim.so')
    for key, value in dict(pattern=str(pattern), samples=20000, slices=5,
                           line_count=4, topic='/livox/lidar', frame_id='mid360_frame',
                           xfer_format=0, range_noise=.02, angular_noise=.15,
                           downsample=1, seed=42, default_reflectivity=100).items():
        element(plugin, key, value)
    # Keep the existing post. Replace only the old sensor housing visual.
    visual = next(v for v in boat.findall('.//visual')
                  if 'lidar_wamv_link_visual' in v.get('name', ''))
    for child in list(visual):
        visual.remove(child)
    element(visual, 'pose', '0.7 -0.3 1.753 0 0 0')
    mesh = element(element(visual, 'geometry'), 'mesh')
    element(mesh, 'uri', str(ROOT/'external/livox_gz_sim/meshes/mid360.dae'))
    ET.indent(tree)
    tree.write(path, encoding='utf-8', xml_declaration=True)
    scene_path = generated/'scene.json'
    scene = json.loads(scene_path.read_text())
    scene['lidar_model'] = 'mid360'
    scene['lidar_frame'] = 'mid360_frame'
    scene['lidar_rpy'] = [0., 0., 0.]
    for key in ('lidar_samples','lidar_beams'):
        scene.pop(key,None)
    scene['lidar_vertical_min_rad']=math.radians(-7.)
    scene['lidar_vertical_max_rad']=math.radians(52.)
    scene['lidar_simulation'] = {
        'method': 'Recorded Livox scan pattern sampled from GPU range images',
        'pattern_sha256': hashlib.sha256(pattern.read_bytes()).hexdigest(),
        'emitted_rays_per_second': 200000, 'publish_hz': 10, 'slices_per_frame': 5,
        'range_m': [.1, 40.], 'nominal_fov_deg': [360., -7., 52.],
        'dense_image_shape': [151, 901], 'range_noise_sigma_m': .02,
        'angular_noise_sigma_deg': .15, 'seed': 42,
        'output': 'PointCloud2 x/y/z/intensity/tag/line/timestamp; height=1; valid returns only',
        'limitations': ['20 ms time slices, not individual-ray physics',
                        'Nearest angular pixel approximation',
                        'No calibrated water reflectivity, rain, spray, multipath or range-dependent detection',
                        'Existing WAM-V sensor mount/inertia retained; nominal housing visual replaced',
                        'MID-360 built-in IMU is not part of this lidar-only adapter']}
    scene_path.write_text(json.dumps(scene, indent=2)+'\n')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('generated', type=Path)
    prepare(parser.parse_args().generated)
