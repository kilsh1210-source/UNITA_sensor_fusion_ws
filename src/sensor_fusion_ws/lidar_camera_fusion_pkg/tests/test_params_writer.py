"""캘리브레이션 도구의 '저장'(s키)이 params.yaml을 안전하게 고치는지 검증.

params.yaml은 설명 주석이 값만큼 중요한 파일이라, safe_load -> safe_dump로
다시 쓰면 안 된다. 값만 제자리에서 갈아끼우는지, 그리고 ROS2가 받아들이는
타입(double 자리에 int가 들어가지 않는지)으로 쓰는지를 본다.
"""

import textwrap

import pytest
import yaml

from lidar_camera_fusion_pkg.params_writer import _format_value, update_node_params


SAMPLE = textwrap.dedent("""\
    # 맨 위 주석
    rplidar_node:
      ros__parameters:
        serial_port: /dev/ttyUSB0   # 포트
        frame_id: laser

    image_fusion_node:
      ros__parameters:
        # 내부 파라미터
        fx: 565.529459
        cy: 290.095566
        calib_pitch_deg: 0.0      # + 면 점이 화면 위로
        calib_yaw_deg: 0.0
        use_urdf_extrinsic: true

    l_shape_node:
      ros__parameters:
        max_range: 6.0
    """)


@pytest.fixture()
def params_path(tmp_path):
    p = tmp_path / 'params.yaml'
    p.write_text(SAMPLE, encoding='utf-8')
    return str(p)


def _fusion(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)['image_fusion_node']['ros__parameters']


def test_updates_existing_keys_in_place(params_path):
    updated, added = update_node_params(
        params_path, 'image_fusion_node',
        {'calib_pitch_deg': -1.75, 'calib_yaw_deg': 0.5})

    assert set(updated) == {'calib_pitch_deg', 'calib_yaw_deg'}
    assert added == []

    values = _fusion(params_path)
    assert values['calib_pitch_deg'] == pytest.approx(-1.75)
    assert values['calib_yaw_deg'] == pytest.approx(0.5)


def test_preserves_comments_and_other_sections(params_path):
    update_node_params(params_path, 'image_fusion_node', {'calib_pitch_deg': -1.75})

    text = open(params_path, encoding='utf-8').read()
    assert '# 맨 위 주석' in text
    assert '# + 면 점이 화면 위로' in text      # 바꾼 줄의 주석이 살아있어야 함
    assert '# 포트' in text
    assert '# 내부 파라미터' in text

    data = yaml.safe_load(text)
    assert data['rplidar_node']['ros__parameters']['serial_port'] == '/dev/ttyUSB0'
    assert data['l_shape_node']['ros__parameters']['max_range'] == 6.0
    assert data['image_fusion_node']['ros__parameters']['fx'] == pytest.approx(565.529459)
    assert data['image_fusion_node']['ros__parameters']['use_urdf_extrinsic'] is True


def test_missing_keys_are_appended_to_section(params_path):
    updated, added = update_node_params(
        params_path, 'image_fusion_node',
        {'calib_pitch_deg': 1.0, 'calib_roll_deg': 0.25, 'calib_height_m': -0.02})

    assert updated == ['calib_pitch_deg']
    assert set(added) == {'calib_roll_deg', 'calib_height_m'}

    values = _fusion(params_path)
    assert values['calib_roll_deg'] == pytest.approx(0.25)
    assert values['calib_height_m'] == pytest.approx(-0.02)
    # 다른 섹션으로 새어나가지 않아야 함
    data = yaml.safe_load(open(params_path, encoding='utf-8'))
    assert 'calib_roll_deg' not in data['l_shape_node']['ros__parameters']


def test_floats_never_written_as_int(params_path):
    """double로 declare된 파라미터에 int가 들어가면 ROS2가 파라미터 타입 오류를 낸다."""
    update_node_params(params_path, 'image_fusion_node',
                       {'calib_pitch_deg': 0.0, 'calib_yaw_deg': 2.0})

    values = _fusion(params_path)
    assert isinstance(values['calib_pitch_deg'], float)
    assert isinstance(values['calib_yaw_deg'], float)


@pytest.mark.parametrize('value,expected', [
    (0.0, '0.0'), (2.0, '2.0'), (-1.75, '-1.75'),
    (0.5, '0.5'), (-0.023, '-0.023'), (14.0, '14.0'),
])
def test_format_value_keeps_decimal_point(value, expected):
    assert _format_value(value) == expected


def test_repeated_save_is_idempotent(params_path):
    values = {'calib_pitch_deg': -1.75, 'calib_yaw_deg': 0.0}
    update_node_params(params_path, 'image_fusion_node', values)
    first = open(params_path, encoding='utf-8').read()
    update_node_params(params_path, 'image_fusion_node', values)
    assert open(params_path, encoding='utf-8').read() == first


def test_backup_is_created(params_path):
    update_node_params(params_path, 'image_fusion_node', {'calib_pitch_deg': 1.0})
    assert open(params_path + '.bak', encoding='utf-8').read() == SAMPLE


def test_unknown_section_raises(params_path):
    with pytest.raises(KeyError):
        update_node_params(params_path, 'no_such_node', {'a': 1.0})
