import importlib
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _load_fusion_module():
    if 'rclpy' not in sys.modules:
        rclpy = types.ModuleType('rclpy')
        class DummyNode:
            def __init__(self, *args, **kwargs):
                pass
            def declare_parameter(self, *args, **kwargs):
                return None
            def get_parameter(self, *args, **kwargs):
                return types.SimpleNamespace(value=None)
            def create_subscription(self, *args, **kwargs):
                return None
            def create_publisher(self, *args, **kwargs):
                return None
            def create_timer(self, *args, **kwargs):
                return None
            def get_logger(self):
                return types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None, error=lambda *a, **k: None)
            def destroy_node(self):
                return None
        rclpy.node = types.ModuleType('rclpy.node')
        rclpy.node.Node = DummyNode
        rclpy.qos = types.ModuleType('rclpy.qos')
        rclpy.qos.qos_profile_sensor_data = object()
        rclpy.init = lambda *args, **kwargs: None
        rclpy.spin = lambda *args, **kwargs: None
        rclpy.shutdown = lambda *args, **kwargs: None
        rclpy.ok = lambda *args, **kwargs: True
        sys.modules['rclpy'] = rclpy
        sys.modules['rclpy.node'] = rclpy.node
        sys.modules['rclpy.qos'] = rclpy.qos

    if 'cv_bridge' not in sys.modules:
        cv_bridge = types.ModuleType('cv_bridge')
        cv_bridge.CvBridge = type('CvBridge', (), {'imgmsg_to_cv2': staticmethod(lambda *a, **k: None), 'cv2_to_imgmsg': staticmethod(lambda *a, **k: None)})
        sys.modules['cv_bridge'] = cv_bridge

    if 'geometry_msgs' not in sys.modules:
        geometry_msgs = types.ModuleType('geometry_msgs')
        msg_mod = types.ModuleType('geometry_msgs.msg')
        msg_mod.TransformStamped = type('TransformStamped', (), {})
        geometry_msgs.msg = msg_mod
        sys.modules['geometry_msgs'] = geometry_msgs
        sys.modules['geometry_msgs.msg'] = msg_mod

    if 'tf2_ros' not in sys.modules:
        tf2_ros = types.ModuleType('tf2_ros')
        tf2_ros.Buffer = type('Buffer', (), {})
        tf2_ros.TransformListener = type('TransformListener', (), {'__init__': lambda *a, **k: None})
        sys.modules['tf2_ros'] = tf2_ros

    if 'sensor_msgs' not in sys.modules:
        sensor_msgs = types.ModuleType('sensor_msgs')
        msg_mod = types.ModuleType('sensor_msgs.msg')
        msg_mod.Image = type('Image', (), {})
        msg_mod.LaserScan = type('LaserScan', (), {})
        sensor_msgs.msg = msg_mod
        sys.modules['sensor_msgs'] = sensor_msgs
        sys.modules['sensor_msgs.msg'] = msg_mod

    if 'std_msgs' not in sys.modules:
        std_msgs = types.ModuleType('std_msgs')
        msg_mod = types.ModuleType('std_msgs.msg')
        msg_mod.Header = type('Header', (), {})
        std_msgs.msg = msg_mod
        sys.modules['std_msgs'] = std_msgs
        sys.modules['std_msgs.msg'] = msg_mod

    if 'interfaces_pkg' not in sys.modules:
        interfaces_pkg = types.ModuleType('interfaces_pkg')
        msg_mod = types.ModuleType('interfaces_pkg.msg')
        msg_mod.DetectionArray = type('DetectionArray', (), {})
        interfaces_pkg.msg = msg_mod
        sys.modules['interfaces_pkg'] = interfaces_pkg
        sys.modules['interfaces_pkg.msg'] = msg_mod

    sys.modules.pop('lidar_camera_fusion_pkg.image_fusion_node', None)
    return importlib.import_module('lidar_camera_fusion_pkg.image_fusion_node')


FusionVisualizerNode = _load_fusion_module().FusionVisualizerNode


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
