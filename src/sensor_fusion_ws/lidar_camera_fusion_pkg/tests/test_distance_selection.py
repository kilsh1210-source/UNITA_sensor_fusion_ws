import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_camera_fusion_pkg.image_fusion_node import FusionVisualizerNode


def test_cluster_average_uses_overlap_points_within_tolerance():
    node = object.__new__(FusionVisualizerNode)
    node.min_range = 0.1
    node.max_range = 10.0
    node.distance_tolerance = 0.6
    node.distance_method = 'center'

    r = np.array([1.0, 1.05, 2.0], dtype=np.float64)
    u = np.array([10, 11, 20], dtype=np.float64)
    v = np.array([20, 21, 30], dtype=np.float64)

    dist, best_uv = node.estimate_distance_in_bbox(u, v, r, 8, 18, 12, 22)

    assert np.isclose(dist, 1.025)
    assert best_uv == (10, 20)


def test_label_position_prefers_box_corner():
    node = object.__new__(FusionVisualizerNode)
    pos = node._compute_distance_label_position(20, 40, 120, 80, 60, 16, 200, 200)

    assert pos[0] >= 20
    assert pos[1] >= 40
