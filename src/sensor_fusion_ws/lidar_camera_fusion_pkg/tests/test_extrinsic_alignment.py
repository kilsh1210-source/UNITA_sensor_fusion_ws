"""라이다->카메라 투영 정렬(extrinsic) 관련 회귀 테스트.

"라이다 점이 실제와 살짝 틀어져 보인다" 문제를 고치면서 만든 것들:

1. TF를 못 쓸 때의 폴백 extrinsic이 URDF/TF 체인과 정확히 일치하는지
   (예전 기본값 cam_height=0.032 / cam_x_offset=0.0 은 URDF와 전혀 달랐고,
    다운틸트도 반영이 안 돼 있어서 폴백으로 떨어지면 조용히 크게 어긋났다)
2. calib_* 미세보정의 부호가 문서에 적힌 대로("+pitch면 점이 화면 위로") 동작하는지
"""

import math
import sys
import types

import numpy as np
import pytest


def _load_node_class():
    """ROS 런타임 없이 image_fusion_node 모듈에서 클래스만 꺼내온다."""
    try:
        from lidar_camera_fusion_pkg.image_fusion_node import FusionVisualizerNode
    except ImportError:  # pragma: no cover - ROS 환경이 아닐 때
        pytest.skip('ROS2 환경(rclpy/cv_bridge/interfaces_pkg)이 필요합니다')
    return FusionVisualizerNode


# --- URDF(unita_minicar.urdf) + description.launch.py 의 TF 체인을 그대로 재현 ---

def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _homog(rot, trans):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = trans
    return m


CAMERA_PITCH_DEG = 14.0

# base_link 기준 장착 위치 (URDF 값)
T_BASE_LASER = _homog(_rot_z(math.pi), [0.500, 0.0, 0.119])
T_BASE_CAMERA = _homog(np.eye(3), [-0.230, 0.0, 0.669])

# camera_link -> camera_link_tilted -> camera_optical_frame_tilted
T_CAMERA_TILTED = _homog(_rot_y(math.radians(CAMERA_PITCH_DEG)), [0.0, 0.0, 0.0])
T_TILTED_OPTICAL = _homog(
    _rot_z(-math.pi / 2) @ _rot_y(0.0) @ _rot_x(-math.pi / 2), [0.0, 0.0, 0.0])

T_BASE_OPTICAL = T_BASE_CAMERA @ T_CAMERA_TILTED @ T_TILTED_OPTICAL
# image_fusion_node가 TF에서 받아오는 것과 동일한 laser -> camera_optical 변환
TF_EXTRINSIC = np.linalg.inv(T_BASE_OPTICAL) @ T_BASE_LASER

# params.yaml의 캘리브레이션 값
K = np.array([[565.529459, 0.0, 337.983746],
              [0.0, 566.767111, 290.095566],
              [0.0, 0.0, 1.0]])


def _project(extrinsic, range_m, angle_deg):
    """laser 프레임의 한 점을 이미지 픽셀 좌표로 투영."""
    a = math.radians(angle_deg)
    p = np.array([range_m * math.cos(a), range_m * math.sin(a), 0.0, 1.0])
    cam = extrinsic @ p
    assert cam[2] > 0, '카메라 뒤쪽 점'
    uv = K @ cam[:3]
    return uv[0] / uv[2], uv[1] / uv[2]


# laser 프레임은 URDF에서 yaw 180도 돌아있으므로, 차량 정면 = laser 각도 180도
STRAIGHT_AHEAD_DEG = 180.0


def test_fallback_extrinsic_matches_urdf_tf_chain():
    """TF가 없을 때 쓰는 폴백이 URDF/TF 체인과 같은 변환을 만들어야 한다."""
    node_cls = _load_node_class()

    fallback = node_cls._init_extrinsic(
        dist=0.730,        # 카메라에서 본 라이다 원점까지 앞쪽 거리
        height=0.550,      # 카메라에서 본 라이다 원점까지 아래쪽 거리
        front_angle_deg=-180.0,
        cam_pitch_deg=CAMERA_PITCH_DEG,
    )

    np.testing.assert_allclose(fallback, TF_EXTRINSIC, atol=1e-9)


def test_fallback_without_pitch_is_wrong():
    """다운틸트를 빼먹으면(예전 동작) 실제로 크게 어긋난다는 것을 명시."""
    node_cls = _load_node_class()

    no_pitch = node_cls._init_extrinsic(0.730, 0.550, -180.0, cam_pitch_deg=0.0)
    _, v_correct = _project(TF_EXTRINSIC, 2.0, STRAIGHT_AHEAD_DEG)
    _, v_no_pitch = _project(no_pitch, 2.0, STRAIGHT_AHEAD_DEG)

    # 14도를 통째로 빼먹으면 2 m 앞 점 기준 100 px 이상 밀린다
    assert abs(v_no_pitch - v_correct) > 100


class _CalibStub:
    """_apply_calibration만 쓰기 위한 최소 스텁 (ROS 노드 생성 회피)."""

    def __init__(self, node_cls, pitch=0.0, yaw=0.0, roll=0.0, height=0.0):
        self.calib_pitch_deg = pitch
        self.calib_yaw_deg = yaw
        self.calib_roll_deg = roll
        self.calib_height_m = height
        self._apply = node_cls._apply_calibration

    def apply(self, ext):
        return self._apply(self, ext)


def test_zero_calibration_is_identity():
    node_cls = _load_node_class()
    out = _CalibStub(node_cls).apply(TF_EXTRINSIC)
    np.testing.assert_allclose(out, TF_EXTRINSIC, atol=1e-12)


def test_positive_pitch_moves_points_up():
    """문서/HUD에 적힌 대로 +pitch(=i키)면 점이 화면 위로 가야 한다."""
    node_cls = _load_node_class()
    _, v_base = _project(TF_EXTRINSIC, 2.0, STRAIGHT_AHEAD_DEG)
    _, v_up = _project(_CalibStub(node_cls, pitch=1.0).apply(TF_EXTRINSIC),
                       2.0, STRAIGHT_AHEAD_DEG)

    assert v_up < v_base                       # 위로 = v 감소
    assert 9.0 < (v_base - v_up) < 11.0        # 1도 ≈ 10 px


def test_positive_yaw_moves_points_right():
    node_cls = _load_node_class()
    u_base, _ = _project(TF_EXTRINSIC, 2.0, STRAIGHT_AHEAD_DEG)
    u_right, _ = _project(_CalibStub(node_cls, yaw=1.0).apply(TF_EXTRINSIC),
                          2.0, STRAIGHT_AHEAD_DEG)

    assert u_right > u_base
    assert 9.0 < (u_right - u_base) < 11.0


def test_positive_height_moves_points_down():
    """+height = 카메라가 실제로 더 높이 달려있다 -> 점이 아래로."""
    node_cls = _load_node_class()
    _, v_base = _project(TF_EXTRINSIC, 2.0, STRAIGHT_AHEAD_DEG)
    _, v_down = _project(_CalibStub(node_cls, height=0.05).apply(TF_EXTRINSIC),
                         2.0, STRAIGHT_AHEAD_DEG)

    assert v_down > v_base


def test_calibration_is_range_independent_for_rotation():
    """회전 보정은 거리와 무관하게 같은 픽셀량만큼 움직여야 한다 (기울기 오차의 특징)."""
    node_cls = _load_node_class()
    corrected = _CalibStub(node_cls, pitch=1.0).apply(TF_EXTRINSIC)

    shifts = []
    for r in (1.0, 2.0, 3.0, 5.0):
        _, v_base = _project(TF_EXTRINSIC, r, STRAIGHT_AHEAD_DEG)
        _, v_corr = _project(corrected, r, STRAIGHT_AHEAD_DEG)
        shifts.append(v_base - v_corr)

    assert max(shifts) - min(shifts) < 0.5
