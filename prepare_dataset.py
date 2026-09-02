#!/usr/bin/env python3
"""
Dataset Preparation Pipeline: Enhance existing LiDAR dataset with tactical features
Maps KITTI semantic classes to 4 tactical defense categories:
  Class 0: Drivable Surface / Ground
  Class 1: Trench / Negative Obstacle / Dips  
  Class 2: Static Obstacles / Bunkers / Terrain
  Class 3: Dynamic Targets / Vehicles / Patrols
"""

import pandas as pd
import numpy as np
import logging
import os
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)


class TacticalClassMapper:
    """Map raw semantic classes to 4 tactical defense categories"""
    
    # Tactical class definitions
    TACTICAL_CLASSES = {
        'DRIVABLE': 0,              # Ground, asphalt, road
        'NEGATIVE_TRENCH': 1,       # Trenches, potholes, dips, curbs
        'STATIC_OBSTACLE': 2,       # Walls, poles, trees, barriers, bunkers
        'DYNAMIC_TARGET': 3         # Vehicles, moving personnel
    }
    
    @staticmethod
    def ensure_balanced_classes(df, num_samples_per_class=3000):
        """
        Ensure balanced representation of all 4 tactical classes.
        Oversample underrepresented classes, undersample overrepresented ones.
        """
        balanced_dfs = []
        
        for class_id, class_name in [(0, 'DRIVABLE'), (1, 'NEGATIVE_TRENCH'), 
                                     (2, 'STATIC_OBSTACLE'), (3, 'DYNAMIC_TARGET')]:
            class_df = df[df['semantic_class_id'] == class_id].copy()
            current_count = len(class_df)
            
            logger.info(f"  {class_name}: {current_count} points")
            
            if current_count == 0:
                logger.warning(f"  WARNING: No samples for class {class_name}, generating synthetic samples...")
                # Generate synthetic samples if class is missing
                class_df = _generate_synthetic_class_samples(df, class_id, class_name, num_samples_per_class)
            elif current_count < num_samples_per_class:
                # Oversample
                class_df = class_df.sample(n=num_samples_per_class, replace=True, random_state=42)
                logger.info(f"    -> Oversampled to {num_samples_per_class} points")
            else:
                # Undersample
                class_df = class_df.sample(n=num_samples_per_class, replace=False, random_state=42)
                logger.info(f"    -> Undersampled to {num_samples_per_class} points")
            
            balanced_dfs.append(class_df)
        
        balanced_df = pd.concat(balanced_dfs, ignore_index=True)
        return balanced_df.sample(frac=1, random_state=42).reset_index(drop=True)


def _generate_synthetic_class_samples(df, class_id, class_name, count):
    """Generate synthetic samples for underrepresented classes"""
    logger.info(f"    Generating {count} synthetic {class_name} samples...")
    
    synthetic_samples = []
    base_stats = {
        'DRIVABLE': {
            'z_range': (0.0, 0.3),
            'intensity_range': (20, 50),
            'elevation_diff_range': (-0.1, 0.1),
        },
        'NEGATIVE_TRENCH': {
            'z_range': (-1.0, 0.0),
            'intensity_range': (10, 30),
            'elevation_diff_range': (-1.5, -0.3),
        },
        'STATIC_OBSTACLE': {
            'z_range': (0.5, 3.0),
            'intensity_range': (40, 80),
            'elevation_diff_range': (0.2, 2.0),
        },
        'DYNAMIC_TARGET': {
            'z_range': (0.3, 2.5),
            'intensity_range': (50, 100),
            'elevation_diff_range': (-0.1, 1.5),
        }
    }
    
    stats = base_stats.get(class_name, base_stats['DRIVABLE'])
    existing_points = df[['x', 'y', 'ring_index']].values
    
    for _ in range(count):
        # Base location from existing points
        base_pt = existing_points[np.random.randint(0, len(existing_points))]
        
        sample = {
            'x': base_pt[0] + np.random.normal(0, 0.2),
            'y': base_pt[1] + np.random.normal(0, 0.2),
            'z': np.random.uniform(*stats['z_range']),
            'intensity': np.random.uniform(*stats['intensity_range']),
            'ring_index': int(base_pt[2]),
            'semantic_class_id': class_id,
            'semantic_class_name': class_name,
            'range_distance': np.sqrt(base_pt[0]**2 + base_pt[1]**2 + stats['z_range'][0]**2),
        }
        synthetic_samples.append(sample)
    
    return pd.DataFrame(synthetic_samples)


def compute_traversability_features(df):
    """
    Compute derived features for tactical traversability assessment:
    - elevation_diff: relative height change (z-coordinate proxy)
    - traversability_score: 0-1 score indicating drivability
      * 1.0 = Drivable flat terrain
      * 0.5-0.9 = Passable with caution
      * 0.0-0.5 = Difficult/dangerous terrain
    """
    logger.info("Computing traversability features...")
    
    # Elevation difference (proxy: z-coordinate with normalization)
    z_values = df['z'].values
    z_min, z_max = z_values.min(), z_values.max()
    z_normalized = (z_values - z_min) / (z_max - z_min + 1e-6)
    
    df['elevation_diff'] = z_normalized - 0.5  # Center around 0
    
    # Traversability score based on class and features
    traversability = np.ones(len(df))
    
    # Drivable surface: high traversability
    drivable_mask = df['semantic_class_id'] == 0
    traversability[drivable_mask] = 0.95 - 0.3 * np.abs(df.loc[drivable_mask, 'elevation_diff'])
    
    # Trench/negative obstacles: very low traversability
    trench_mask = df['semantic_class_id'] == 1
    traversability[trench_mask] = 0.1
    
    # Static obstacles: medium-low traversability
    static_mask = df['semantic_class_id'] == 2
    intensity_norm = df.loc[static_mask, 'intensity'] / 100.0
    traversability[static_mask] = 0.3 + 0.2 * (1 - intensity_norm)
    
    # Dynamic targets: treat as obstacles
    dynamic_mask = df['semantic_class_id'] == 3
    traversability[dynamic_mask] = 0.2
    
    # Clamp to [0, 1]
    df['traversability_score'] = np.clip(traversability, 0.0, 1.0)
    
    logger.info(f"  Traversability scores - Min: {df['traversability_score'].min():.3f}, Max: {df['traversability_score'].max():.3f}, Mean: {df['traversability_score'].mean():.3f}")
    
    return df


def prepare_dataset(input_csv='dataset.csv', output_csv='dataset.csv', 
                   samples_per_class=3000, balance=True):
    """
    Main pipeline: load, enhance, and balance dataset for training
    """
    logger.info("=" * 70)
    logger.info("LiDAR Dataset Preparation Pipeline")
    logger.info("=" * 70)
    
    # Load existing dataset
    logger.info(f"Loading dataset from {input_csv}...")
    if not os.path.exists(input_csv):
        logger.error(f"Input file {input_csv} not found!")
        return False
    
    df = pd.read_csv(input_csv)
    logger.info(f"Loaded {len(df)} points from {input_csv}")
    
    # Verify required columns
    required_cols = ['x', 'y', 'z', 'intensity', 'semantic_class_id', 'semantic_class_name']
    if not all(col in df.columns for col in required_cols):
        logger.error(f"Missing required columns. Found: {df.columns.tolist()}")
        return False
    
    logger.info(f"Dataset columns: {df.columns.tolist()}")
    
    # Log class distribution before processing
    logger.info("\nClass distribution (before processing):")
    for class_id in sorted(df['semantic_class_id'].unique()):
        class_name = df[df['semantic_class_id'] == class_id]['semantic_class_name'].iloc[0]
        count = len(df[df['semantic_class_id'] == class_id])
        logger.info(f"  Class {class_id} ({class_name}): {count} points")
    
    # Balance classes
    if balance:
        logger.info(f"\nBalancing dataset (target: {samples_per_class} points per class)...")
        df = TacticalClassMapper.ensure_balanced_classes(df, samples_per_class)
    
    # Compute traversability features
    df = compute_traversability_features(df)
    
    # Log final class distribution
    logger.info("\nClass distribution (after processing):")
    for class_id in sorted(df['semantic_class_id'].unique()):
        class_name = df[df['semantic_class_id'] == class_id]['semantic_class_name'].iloc[0]
        count = len(df[df['semantic_class_id'] == class_id])
        logger.info(f"  Class {class_id} ({class_name}): {count} points")
    
    # Save enhanced dataset
    output_cols = ['x', 'y', 'z', 'intensity', 'ring_index', 'semantic_class_id', 
                   'semantic_class_name', 'range_distance', 'elevation_diff', 'traversability_score']
    
    # Ensure all columns exist (add missing ones)
    for col in output_cols:
        if col not in df.columns:
            if col == 'ring_index':
                df[col] = 0
            elif col == 'range_distance':
                df[col] = np.sqrt(df['x']**2 + df['y']**2 + df['z']**2)
    
    df_output = df[output_cols].copy()
    df_output.to_csv(output_csv, index=False)
    
    logger.info(f"\n✓ Enhanced dataset saved to {output_csv}")
    logger.info(f"  Total points: {len(df_output)}")
    logger.info(f"  Features: {', '.join(output_cols)}")
    logger.info("=" * 70)
    
    return True


if __name__ == '__main__':
    success = prepare_dataset(
        input_csv='dataset.csv',
        output_csv='dataset.csv',
        samples_per_class=3000,
        balance=True
    )
    
    if not success:
        exit(1)
