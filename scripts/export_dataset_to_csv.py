#!/usr/bin/env python3
"""
Tactical LiDAR & Semantic Dataset Exporter to CSV.

Exports full synthetic LiDAR point cloud frames with:
- Cartesian coordinates: x, y, z (meters)
- Intensity / Reflectance
- Laser Ring Index (0-31 for ground scan, 32+ for obstacles)
- Semantic Class ID (0: Drivable, 1: Negative Obstacle/Trench, 2: Static Obstacle, 3: Dynamic Target)
- Semantic Class Name
- Spherical Range Distance (meters)
"""

import argparse
import csv
import math
import os
import sys
import numpy as np

SEMANTIC_NAMES = {
  0: 'DRIVABLE',
  1: 'NEGATIVE_TRENCH',
  2: 'STATIC_OBSTACLE',
  3: 'DYNAMIC_TARGET',
}


def generate_lidar_frame(time_phase: float = 0.0):
  """Generates a complete tactical LiDAR point cloud frame with ground truth semantics."""
  records = []

  # 1. Multi-Ring Ground Plane (32 laser rings x 360 rays)
  num_rings = 32
  rays_per_ring = 360
  radii = np.linspace(0.8, 60.0, num_rings)
  angles = np.linspace(0, 2 * np.pi, rays_per_ring, endpoint=False)

  for ring_idx, r in enumerate(radii):
    for a in angles:
      x = float(r * math.cos(a))
      y = float(r * math.sin(a))
      z = 0.02
      intensity = 30.0
      class_id = 0

      # Road Curbs at y = ±3.5m
      if 3.3 <= abs(y) <= 3.7 and -5.0 < x < 40.0:
        z = 0.18
        intensity = 75.0
        class_id = 1  # Non-drivable curb

      # Negative Obstacle 1: Trench at x in [5.5, 6.8], y in [-3.0, 3.0]
      if 5.5 <= x <= 6.8 and abs(y) <= 3.0:
        z = -0.45
        intensity = 10.0
        class_id = 1  # Negative Obstacle / Trench

      # Negative Obstacle 2: Road Pothole at (3.2m, 1.2m, r=0.6m)
      if math.hypot(x - 3.2, y - 1.2) <= 0.6:
        z = -0.22
        intensity = 15.0
        class_id = 1  # Negative Obstacle / Pothole

      dist = math.sqrt(x * x + y * y + z * z)
      records.append((
          round(x, 4),
          round(y, 4),
          round(z, 4),
          round(intensity, 2),
          int(ring_idx),
          int(class_id),
          SEMANTIC_NAMES[class_id],
          round(dist, 4),
      ))

  # 2. Static Obstacles (Blast Wall at x in [12, 16], y = -4.5m)
  for wx in np.linspace(12.0, 16.0, 25):
    for wz in np.linspace(0.0, 1.4, 15):
      x, y, z = float(wx), -4.5, float(wz)
      dist = math.sqrt(x * x + y * y + z * z)
      records.append((
          round(x, 4),
          round(y, 4),
          round(z, 4),
          90.0,
          32,
          2,
          SEMANTIC_NAMES[2],
          round(dist, 4),
      ))

  # 3. Security Perimeter Poles
  for p_idx, (px, py) in enumerate([
      (22.0, -8.0),
      (22.0, 8.0),
      (45.0, -12.0),
      (45.0, 12.0),
      (65.0, -15.0),
      (65.0, 15.0),
  ]):
    for pz in np.linspace(0.0, 3.5, 20):
      x, y, z = float(px), float(py), float(pz)
      dist = math.sqrt(x * x + y * y + z * z)
      records.append((
          round(x, 4),
          round(y, 4),
          round(z, 4),
          120.0,
          33 + p_idx,
          2,
          SEMANTIC_NAMES[2],
          round(dist, 4),
      ))

  # 4. Dynamic Obstacle 1: Moving Patrol Vehicle at x(t) in [16, 32m], y = 2.0m
  veh_x = 22.0 + 7.0 * math.sin(0.4 * time_phase)
  veh_y = 2.0
  for vx in np.linspace(-1.8, 1.8, 16):
    for vy in np.linspace(-0.9, 0.9, 10):
      for vz in np.linspace(0.2, 1.5, 8):
        x = float(veh_x + vx)
        y = float(veh_y + vy)
        z = float(vz)
        dist = math.sqrt(x * x + y * y + z * z)
        records.append((
            round(x, 4),
            round(y, 4),
            round(z, 4),
            180.0,
            40,
            3,
            SEMANTIC_NAMES[3],
            round(dist, 4),
        ))

  # 5. Dynamic Obstacle 2: Crossing Dismounted Pedestrian at x = 8.5m
  ped_x = 8.5
  ped_y = 2.8 * math.sin(0.6 * time_phase)
  for px in np.linspace(-0.25, 0.25, 6):
    for py in np.linspace(-0.25, 0.25, 6):
      for pz in np.linspace(0.0, 1.75, 14):
        x = float(ped_x + px)
        y = float(ped_y + py)
        z = float(pz)
        dist = math.sqrt(x * x + y * y + z * z)
        records.append((
            round(x, 4),
            round(y, 4),
            round(z, 4),
            220.0,
            41,
            3,
            SEMANTIC_NAMES[3],
            round(dist, 4),
        ))

  # 6. Overpass Clearance Canopy at x = 18.0m, z = 2.2m
  for cx in np.linspace(17.5, 18.5, 8):
    for cy in np.linspace(-4.0, 4.0, 24):
      x, y, z = float(cx), float(cy), 2.2
      dist = math.sqrt(x * x + y * y + z * z)
      records.append((
          round(x, 4),
          round(y, 4),
          round(z, 4),
          85.0,
          42,
          2,
          SEMANTIC_NAMES[2],
          round(dist, 4),
      ))

  return records


def export_to_csv(output_path: str, time_phase: float = 0.0) -> None:
  os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
  records = generate_lidar_frame(time_phase=time_phase)

  headers = [
      'x',
      'y',
      'z',
      'intensity',
      'ring_index',
      'semantic_class_id',
      'semantic_class_name',
      'range_distance',
  ]

  with open(output_path, mode='w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(headers)
    writer.writerows(records)

  # Calculate class statistics
  class_counts = {name: 0 for name in SEMANTIC_NAMES.values()}
  for row in records:
    class_counts[row[6]] += 1

  file_size_kb = os.path.getsize(output_path) / 1024.0

  print('=' * 65)
  print('TACTICAL LIDAR DATASET EXPORT COMPLETE')
  print('=' * 65)
  print(f'Output Path:    {output_path}')
  print(f'Total Points:   {len(records):,}')
  print(f'File Size:      {file_size_kb:.1f} KB ({file_size_kb / 1024.0:.2f} MB)')
  print('-' * 65)
  print('Semantic Class Distribution:')
  for name, count in class_counts.items():
    pct = (count / len(records)) * 100.0
    print(f'  • {name:<20}: {count:>6,} points ({pct:5.1f}%)')
  print('=' * 65)


if __name__ == '__main__':
  default_output = os.path.expanduser(
      '~/robot-dashboard/data/sample_lidar_dataset.csv'
  )
  parser = argparse.ArgumentParser(
      description='Export Tactical LiDAR Point Cloud to CSV'
  )
  parser.add_argument(
      '--output',
      '-o',
      default=default_output,
      help=f'Output CSV file path (default: {default_output})',
  )
  parser.add_argument(
      '--phase',
      '-p',
      type=float,
      default=0.0,
      help='Time phase offset for dynamic objects (default: 0.0)',
  )
  args = parser.parse_args()

  export_to_csv(args.output, args.phase)

