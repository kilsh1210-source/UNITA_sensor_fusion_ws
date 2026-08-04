import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_camera_fusion_pkg.image_fusion_node import FusionVisualizerNode


def test_center_bias_prefers_nearby_points_in_bbox():
    node = FusionVisualizerNode.__new__(FusionVisualizerNode)
    node.distance_method = 'center'

    u = np.array([10, 20, 30, 40], dtype=np.int32)
    v = np.array([10, 20, 30, 40], dtype=np.int32)
    ranges = np.array([6.0, 1.2, 3.5, 2.1], dtype=np.float64)

    dist, best_uv = node.estimate_distance_in_bbox(u, v, ranges, 5, 5, 35, 35)

    assert dist == 1.2
    assert best_uv == (20, 20)


def test_center_bias_ignores_far_background_points_when_center_is_available():
    node = FusionVisualizerNode.__new__(FusionVisualizerNode)
    node.distance_method = 'center'

    u = np.array([10, 20, 30, 40], dtype=np.int32)
    v = np.array([10, 20, 30, 40], dtype=np.int32)
    ranges = np.array([6.0, 1.2, 1.3, 3.5], dtype=np.float64)

    dist, best_uv = node.estimate_distance_in_bbox(u, v, ranges, 5, 5, 45, 45)

    assert dist == 1.2
    assert best_uv == (20, 20)


import numpy as np
