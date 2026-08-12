#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""params.yaml의 특정 노드 파라미터 값만 제자리에서 갈아끼우는 유틸.

`yaml.safe_load` -> `yaml.safe_dump`로 다시 쓰면 주석과 순서가 전부 날아간다.
params.yaml은 설명 주석이 값만큼 중요한 파일이라, 여기서는 줄 단위로 해당
`key: value` 부분만 찾아 바꾼다 (주석/들여쓰기/나머지 줄은 그대로 유지).

캘리브레이션 도구가 "저장" 키 한 번으로 값을 영구 반영하는 데 쓴다.
"""

import os
import re
import shutil
from typing import Dict, List, Optional, Tuple


def find_params_file(explicit: str = '') -> Optional[str]:
    """수정할 params.yaml 경로를 찾는다.

    install/ 아래 복사본을 고치면 다음 colcon build 때 덮여서 날아가므로,
    **소스 트리의 params.yaml을 우선**으로 찾는다.
    """
    if explicit:
        return explicit if os.path.isfile(explicit) else None

    rel = os.path.join('src', 'sensor_fusion_ws', 'sensor_fusion_bringup',
                       'config', 'params.yaml')

    # 1) 현재 위치에서 위로 올라가며 워크스페이스 루트 탐색
    here = os.path.abspath(os.getcwd())
    while True:
        candidate = os.path.join(here, rel)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent

    # 2) 이 파일 위치 기준으로 소스 트리 역추적
    #    .../src/sensor_fusion_ws/lidar_camera_fusion_pkg/lidar_camera_fusion_pkg/params_writer.py
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(6):
        here = os.path.dirname(here)
        candidate = os.path.join(here, rel)
        if os.path.isfile(candidate):
            return candidate

    # 3) 마지막 수단: 설치된 share 경로 (rebuild하면 덮어써짐)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory('sensor_fusion_bringup'), 'config', 'params.yaml')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    return None


def _format_value(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        # 소수점 이하를 전부 깎아 '0'이나 '2'처럼 쓰면 YAML이 int로 읽고,
        # 노드는 double로 declare했으므로 ROS2가 파라미터 타입 오류를 낸다.
        # 반드시 소수점 한 자리는 남긴다.
        text = f'{value:.4f}'.rstrip('0')
        return text + '0' if text.endswith('.') else text
    return str(value)


def update_node_params(path: str, node_name: str, values: Dict[str, object],
                       backup: bool = True) -> Tuple[List[str], List[str]]:
    """`node_name:` 섹션 안의 키들을 values로 갱신한다.

    - 이미 있는 키는 값만 교체 (그 줄의 주석은 유지)
    - 없는 키는 섹션 끝에 추가
    반환: (갱신된 키 목록, 새로 추가된 키 목록)
    """
    with open(path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # 1) `node_name:` 섹션 (들여쓰기 0에서 시작하는 다음 키 전까지)
    start = None
    for i, line in enumerate(lines):
        if re.match(rf'^{re.escape(node_name)}\s*:\s*(#.*)?$', line):
            start = i
            break
    if start is None:
        raise KeyError(f"'{node_name}:' 섹션을 {path} 에서 찾지 못했습니다")

    section_end = len(lines)
    for i in range(start + 1, len(lines)):
        stripped = lines[i].rstrip('\n')
        if stripped and not stripped[0].isspace():
            section_end = i
            break

    # 2) 그 안의 `ros__parameters:` 블록. 실제 파라미터는 전부 여기 아래에 있다.
    #    (예전에는 이걸 안 찾고 섹션의 첫 들여쓰기를 썼더니, 새 키가
    #     ros__parameters의 형제로 들어가서 노드가 읽지 못했다)
    ros_params_line = None
    for i in range(start + 1, section_end):
        if re.match(r'^(\s+)ros__parameters\s*:\s*(#.*)?$', lines[i]):
            ros_params_line = i
            break

    if ros_params_line is None:
        block_start, block_end = start + 1, section_end
        outer_indent = ''
    else:
        outer_indent = re.match(r'^(\s*)', lines[ros_params_line]).group(1)
        block_start = ros_params_line + 1
        block_end = section_end
        for i in range(block_start, section_end):
            stripped = lines[i].rstrip('\n')
            if not stripped.strip():
                continue
            indent = re.match(r'^(\s*)', stripped).group(1)
            if len(indent) <= len(outer_indent):
                block_end = i
                break

    # 3) 파라미터 줄의 들여쓰기 (없으면 ros__parameters보다 두 칸 더)
    param_indent = outer_indent + '  '
    for i in range(block_start, block_end):
        m = re.match(r'^(\s+)[A-Za-z_][A-Za-z0-9_]*\s*:', lines[i])
        if m:
            param_indent = m.group(1)
            break

    updated: List[str] = []
    remaining = dict(values)

    for i in range(block_start, block_end):
        m = re.match(r'^(\s+)([A-Za-z_][A-Za-z0-9_]*)(\s*:\s*)(.*?)(\s*#.*)?$',
                     lines[i].rstrip('\n'))
        if not m:
            continue
        indent, key, sep, _old, comment = m.groups()
        if key not in remaining:
            continue
        new_value = _format_value(remaining.pop(key))
        lines[i] = f'{indent}{key}{sep}{new_value}{comment or ""}\n'
        updated.append(key)

    added: List[str] = []
    if remaining:
        # ros__parameters 블록의 마지막 내용 줄 바로 뒤에 삽입
        insert_at = block_end
        while insert_at - 1 >= block_start and not lines[insert_at - 1].strip():
            insert_at -= 1
        block = [f'{param_indent}{k}: {_format_value(v)}\n' for k, v in remaining.items()]
        lines[insert_at:insert_at] = block
        added = list(remaining.keys())

    if backup:
        shutil.copy2(path, path + '.bak')

    with open(path, 'w', encoding='utf-8') as f:
        f.writelines(lines)

    return updated, added
