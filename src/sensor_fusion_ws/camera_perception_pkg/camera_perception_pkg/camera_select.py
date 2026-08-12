#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""연결된 카메라 중 원하는 한 대를 안정적으로 골라내는 유틸.

카메라를 두 대 이상 꽂으면 `/dev/video*` 번호가 부팅/재연결마다 바뀐다.
그래서 `cv2.VideoCapture(0)`처럼 숫자로 열면 어떤 날은 YOLO용 전방 카메라가,
어떤 날은 다른 카메라가 잡힌다.

해결책은 번호 대신 **바뀌지 않는 식별자**로 고르는 것:

- `/dev/v4l/by-id/...`   : USB 장치의 제조사/모델/시리얼 기반. 카메라를 다른 포트에
                           꽂아도 그대로 따라간다. **모델이 서로 다르면 이걸 쓴다.**
- `/dev/v4l/by-path/...` : 꽂힌 USB 포트 기반. **같은 모델 두 대**를 구분할 때 쓴다
                           (대신 포트를 바꿔 꽂으면 달라짐).

또 하나의 함정: 카메라 한 대가 `/dev/video0`, `/dev/video1`처럼 노드를 2개 만드는 경우가
흔하다(영상용 + 메타데이터용). 뒤쪽 번호는 열려도 프레임이 안 나온다. `is_capture_device()`가
이걸 걸러낸다.
"""

import glob
import os
import re
from typing import Dict, List, Optional


V4L_BY_ID_DIR = '/dev/v4l/by-id'
V4L_BY_PATH_DIR = '/dev/v4l/by-path'


def _read_sysfs_name(device: str) -> str:
    """/sys/class/video4linux/videoN/name 에서 사람이 읽는 카메라 이름을 얻는다."""
    m = re.match(r'^/dev/(video\d+)$', device)
    if not m:
        return ''
    try:
        with open(f'/sys/class/video4linux/{m.group(1)}/name', encoding='utf-8') as f:
            return f.read().strip()
    except OSError:
        return ''


def is_capture_device(device: str) -> bool:
    """실제로 영상이 나오는 노드인지 확인 (메타데이터 전용 노드를 걸러냄).

    v4l2 capability를 ioctl로 읽는 대신, sysfs의 device capabilities를 본다.
    읽을 수 없으면 True로 두고 실제 열기 단계에서 판정하게 한다.
    """
    m = re.match(r'^/dev/(video\d+)$', device)
    if not m:
        return True
    try:
        path = f'/sys/class/video4linux/{m.group(1)}/device/../'
        del path  # sysfs 경로는 커널 버전마다 달라서 신뢰하지 않음
        with open(f'/sys/class/video4linux/{m.group(1)}/index', encoding='utf-8') as f:
            # index 0 이 보통 영상 노드, 1 이상은 메타데이터인 경우가 많다.
            # 절대적인 규칙은 아니라 참고용으로만 쓴다.
            return f.read().strip() == '0'
    except OSError:
        return True


def _symlinks_for(directory: str) -> Dict[str, str]:
    """{/dev/videoN: 심볼릭링크 경로} 매핑."""
    out: Dict[str, str] = {}
    if not os.path.isdir(directory):
        return out
    for link in sorted(glob.glob(os.path.join(directory, '*'))):
        try:
            target = os.path.realpath(link)
        except OSError:
            continue
        # 한 장치에 링크가 여러 개면 먼저 나온 것(보통 index0)을 쓴다
        out.setdefault(target, link)
    return out


def list_video_devices() -> List[Dict[str, object]]:
    """연결된 모든 /dev/video* 를 안정 식별자와 함께 나열."""
    by_id = _symlinks_for(V4L_BY_ID_DIR)
    by_path = _symlinks_for(V4L_BY_PATH_DIR)

    devices = []
    for device in sorted(glob.glob('/dev/video*'),
                         key=lambda d: int(re.sub(r'\D', '', d) or 0)):
        devices.append({
            'device': device,
            'index': int(re.sub(r'\D', '', device) or 0),
            'name': _read_sysfs_name(device),
            'by_id': by_id.get(device, ''),
            'by_path': by_path.get(device, ''),
            'likely_capture': is_capture_device(device),
        })
    return devices


def probe_capture(device: str, timeout_frames: int = 3) -> bool:
    """실제로 열어서 프레임이 나오는지 확인 (느리므로 목록 출력 시에만 사용)."""
    import cv2
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            return False
        for _ in range(timeout_frames):
            ok, frame = cap.read()
            if ok and frame is not None and frame.size:
                return True
        return False
    finally:
        cap.release()


def resolve_camera(device: str = '', name: str = '',
                   index: int = 0) -> tuple:
    """열어야 할 카메라를 결정한다. 우선순위: device > name > index.

    반환: (opencv에 넘길 값, 사람이 읽는 설명, 경고 목록)

    - device: '/dev/video2' 또는 '/dev/v4l/by-id/...' 같은 명시적 경로
    - name  : by-id 링크 이름이나 sysfs 카메라 이름의 **부분 문자열**
              (예: 'C920', 'HD_Pro'). 대소문자 구분 없음.
    - index : 위 둘이 비었을 때 쓰는 기존 방식의 숫자 번호
    """
    warnings: List[str] = []
    devices = list_video_devices()

    # 1) 명시적 경로
    if device:
        if not os.path.exists(device):
            warnings.append(
                f"camera_device='{device}' 가 존재하지 않습니다. "
                f"연결된 장치: {[d['device'] for d in devices]}")
            return device, device, warnings
        real = os.path.realpath(device)
        return device, f'{device} -> {real}', warnings

    # 2) 이름(부분 문자열)로 찾기
    if name:
        needle = name.lower()
        matches = []
        for d in devices:
            haystack = ' '.join([
                os.path.basename(str(d['by_id'])),
                os.path.basename(str(d['by_path'])),
                str(d['name']),
            ]).lower()
            if needle in haystack:
                matches.append(d)

        # 영상 노드로 보이는 것 우선
        capture_matches = [d for d in matches if d['likely_capture']]
        chosen_pool = capture_matches or matches

        if not chosen_pool:
            warnings.append(
                f"camera_name='{name}' 과 일치하는 카메라가 없습니다. "
                f"연결된 카메라: "
                f"{[(d['device'], d['name']) for d in devices]}")
        else:
            if len(chosen_pool) > 1:
                warnings.append(
                    f"camera_name='{name}' 에 {len(chosen_pool)}개가 일치해서 "
                    f"첫 번째({chosen_pool[0]['device']})를 씁니다. "
                    f"더 구체적인 이름을 쓰거나 camera_device로 지정하세요: "
                    f"{[d['device'] for d in chosen_pool]}")
            chosen = chosen_pool[0]
            target = str(chosen['by_id'] or chosen['by_path'] or chosen['device'])
            desc = f"{target} -> {chosen['device']} ({chosen['name']})"
            return target, desc, warnings

    # 3) 숫자 번호 (기존 방식)
    capture_devices = [d for d in devices if d['likely_capture']]
    if len(capture_devices) > 1:
        warnings.append(
            f"카메라가 {len(capture_devices)}대 연결돼 있는데 번호(cam_num={index})로 "
            f"고르고 있습니다. /dev/video 번호는 재부팅·재연결마다 바뀔 수 있으니 "
            f"camera_name 또는 camera_device로 고정하세요. "
            f"후보: {[(d['device'], d['name'], os.path.basename(str(d['by_id']))) for d in capture_devices]}")

    return index, f'index {index}', warnings


def format_device_table(probe: bool = False) -> str:
    """연결된 카메라를 표로 출력 (list_cameras 콘솔 스크립트용)."""
    devices = list_video_devices()
    if not devices:
        return '연결된 /dev/video* 장치가 없습니다.'

    lines = ['연결된 카메라:', '']
    for d in devices:
        mark = '  ' if d['likely_capture'] else ' x'
        lines.append(f"{mark} {d['device']}  (cam_num: {d['index']})")
        if d['name']:
            lines.append(f"      이름     : {d['name']}")
        if d['by_id']:
            lines.append(f"      by-id    : {d['by_id']}")
        if d['by_path']:
            lines.append(f"      by-path  : {d['by_path']}")
        if probe:
            ok = probe_capture(d['device'])
            lines.append(f"      프레임   : {'나옴 O' if ok else '안 나옴 X'}")
        lines.append('')

    lines += [
        'x 표시는 메타데이터 전용 노드일 가능성이 높습니다(영상 안 나옴).',
        '',
        'params.yaml 의 image_publisher_node 에 아래처럼 고정하세요:',
        '',
        '    camera_name: "<위 by-id 이름의 일부, 예: C920>"',
        '  또는',
        '    camera_device: "/dev/v4l/by-id/<위 by-id 전체 경로>"',
        '',
        '모델이 같은 카메라 2대라면 by-id로는 구분이 어려우니 by-path(꽂은 포트 기준)를 쓰세요.',
    ]
    return '\n'.join(lines)


def main():
    """ros2 run camera_perception_pkg list_cameras"""
    import sys
    probe = '--probe' in sys.argv or '-p' in sys.argv
    print(format_device_table(probe=probe))
    if not probe:
        print('실제로 영상이 나오는지까지 확인하려면: --probe 옵션을 붙이세요.')


if __name__ == '__main__':
    main()
