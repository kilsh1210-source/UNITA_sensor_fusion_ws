#!/usr/bin/env bash
# 실행 중인 ros2 launch(및 그 자식 노드들)를 안전하게 종료한다.
#
# rplidar_node가 "이전 launch가 아직 안 죽었는데 새 launch가 뜨면서 /dev/ttyUSB0를
# 못 열어 그 자리에서 죽는" 문제(SDK 에러 0x80008004)가 반복됐다. 원인은 Ctrl+C 후
# 셸 프롬프트가 돌아오기 전에(=이전 launch 프로세스 트리가 아직 살아있는데) 재launch한
# 것이었다. 이 스크립트는 재launch 전에 항상 먼저 실행해서, 이전 launch가 실제로
# 완전히 죽었는지 확인/보장한다.
#
# 사용법: ./stop_drive.sh   (그 다음에 ros2 launch ... 실행)

set -uo pipefail

PIDS=$(pgrep -f "ros2 launch" || true)
if [ -z "$PIDS" ]; then
    echo "실행 중인 ros2 launch 없음."
else
    echo "종료 대상 (ros2 launch) PID: $PIDS"
    kill -INT $PIDS 2>/dev/null || true

    # 최대 15초 동안 정상 종료(SIGINT)를 기다린다. YOLO 등 GPU 노드는 정리에 시간이
    # 좀 걸릴 수 있어서 너무 짧게 잡지 않는다.
    for i in $(seq 1 15); do
        sleep 1
        REMAIN=$(pgrep -f "ros2 launch" || true)
        if [ -z "$REMAIN" ]; then
            echo "정상 종료됨 (${i}s)"
            break
        fi
        if [ "$i" -eq 15 ]; then
            echo "15초 지나도 안 죽어서 강제 종료(SIGKILL)"
            kill -KILL $REMAIN 2>/dev/null || true
        fi
    done
fi

# launch 트리 밖으로 떨어져 나가 orphan된 노드가 혹시 남아있으면 마저 정리한다
# (예: 터미널을 그냥 닫아서 launch 프로세스만 죽고 자식은 계속 돈 경우).
NODE_NAMES="rplidar_node image_publisher_node yolov8_node bird_eye_node image_fusion_node \
l_shape_node lane_info_extractor_node path_planner_node motion_planner_node \
serial_sender_node robot_state_publisher static_transform_publisher"

for name in $NODE_NAMES; do
    P=$(pgrep -f "$name" || true)
    if [ -n "$P" ]; then
        echo "잔여 프로세스 정리: $name ($P)"
        kill -KILL $P 2>/dev/null || true
    fi
done

sleep 0.5
echo
if command -v fuser >/dev/null 2>&1 && fuser /dev/ttyUSB0 >/dev/null 2>&1; then
    echo "경고: /dev/ttyUSB0가 여전히 점유 중입니다. 재launch 전에 확인하세요."
else
    echo "라이다 포트(/dev/ttyUSB0) 비어있음 — 재launch 가능."
fi
