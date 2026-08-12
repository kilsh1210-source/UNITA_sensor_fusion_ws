#!/usr/bin/env python3
"""조향 최소 구동 PWM과 실제 기계적 가동범위를 측정한다.

펌웨어의 임시 개루프 명령 "F,<pwm>"을 쓴다. 명령 스트림이 끊기면 아두이노가
FRONT_TEST_WD_MS(300ms) 안에 앞모터를 멈추므로, Ctrl+C를 누르면 바로 정지한다.

측정할 때마다 반드시 중앙(pot 510)으로 복귀한 뒤에 민다. 안 그러면 이동량이
"실제 가동범위"가 아니라 "직전 위치에서 중앙까지의 거리"가 되어 무의미해진다.

  python3 steer_pwm_sweep.py

단계 3개로 진행한다.
  1) 조향이 움직이기 시작하는 최소 PWM 탐색
  2) PWM별 이동량 측정 (매번 중앙 복귀 후)
  3) 실제 기계적 한계(pot 최소/최대) 측정

바퀴가 스토퍼에 물려 소리가 나면 Ctrl+C.
"""

import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 115200

CENTER = 510             # 자로 재서 직진 맞춘 뒤 실측한 pot 값
CENTER_TOL = 8           # 중앙 복귀 허용 오차. 조향 분해능보다 작으면 진동한다
RECENTER_MARGIN = 30     # 복귀용 PWM = 최소구동PWM + 이 값

SEARCH_STEPS = [60, 80, 100, 120, 140, 160, 180, 200]
MEASURE_SEC = 1.0        # PWM별 이동량 측정 시 미는 시간
LIMIT_STALL_SEC = 1.0    # 이 시간 동안 pot이 안 변하면 기계적 한계로 간주


class Aborted(Exception):
    pass


def read_pot(ser, duration=0.15):
    """S <값> 라인에서 조향 포텐셔미터 값을 읽는다. 없으면 None."""
    last = None
    t0 = time.time()
    while time.time() - t0 < duration:
        line = ser.readline().strip()
        if line.startswith(b'S ') and b':' not in line:
            try:
                last = int(line[2:])
            except ValueError:
                pass
    return last


def pot_now(ser, tries=6):
    for _ in range(tries):
        v = read_pot(ser, 0.15)
        if v is not None:
            return v
    raise Aborted("pot 값을 읽지 못했습니다 (아두이노 응답 없음)")


def stop(ser):
    for _ in range(3):
        ser.write(b"F,0\n")
        time.sleep(0.03)


def push_for(ser, pwm, seconds):
    """pwm으로 seconds 동안 민다. (시작pot, 끝pot) 반환."""
    start = pot_now(ser)
    t0 = time.time()
    while time.time() - t0 < seconds:
        ser.write(f"F,{pwm}\n".encode())
        time.sleep(0.05)
        read_pot(ser, 0.03)
    stop(ser)
    time.sleep(0.4)
    return start, pot_now(ser)


def push_until_stuck(ser, pwm, max_sec=12.0):
    """pot이 LIMIT_STALL_SEC 동안 안 변할 때까지 민다. (시작, 끝, 도달여부) 반환."""
    start = pot_now(ser)
    prev = start
    last_move = time.time()
    t0 = time.time()
    while time.time() - t0 < max_sec:
        ser.write(f"F,{pwm}\n".encode())
        time.sleep(0.05)
        cur = read_pot(ser, 0.05)
        if cur is not None:
            if abs(cur - prev) >= 2:
                last_move = time.time()
                prev = cur
            elif time.time() - last_move > LIMIT_STALL_SEC:
                stop(ser)
                time.sleep(0.3)
                return start, pot_now(ser), True
    stop(ser)
    time.sleep(0.3)
    return start, pot_now(ser), False


def pulse(ser, pwm, seconds):
    """지정 시간만큼만 밀고 확실히 멈춘다."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        ser.write(f"F,{pwm}\n".encode())
        time.sleep(0.02)
    stop(ser)


def goto_center(ser, pwm, sign_up, timeout=25.0):
    """개루프로 pot을 CENTER 근처까지 되돌린다. sign_up: pot을 키우는 PWM 부호.

    반드시 '멈춤 -> 안정화 -> 측정 -> 짧은 펄스' 순서로 돈다.
    모터를 돌린 채로 측정하면 워치독(300ms) 동안 계속 밀려서 목표를 지나치고,
    반대로 밀기를 반복하며 중앙에서 진동한다.
    """
    t0 = time.time()
    burst = 0.06
    prev_err = None
    overshoots = 0

    while time.time() - t0 < timeout:
        stop(ser)
        time.sleep(0.25)                 # 관성까지 멎기를 기다린 뒤 측정
        pot = pot_now(ser)
        err = CENTER - pot

        if abs(err) <= CENTER_TOL:
            return pot

        if prev_err is not None:
            if abs(err) >= abs(prev_err) - 1:
                burst = min(burst * 1.6, 0.30)   # 안 움직였으면 펄스를 늘린다
            if err * prev_err < 0:
                overshoots += 1
                burst = max(burst * 0.5, 0.03)   # 지나쳤으면 펄스를 줄인다
                if overshoots >= 4:
                    print(f"    (중앙 ±{abs(err)} 이내로는 더 못 좁힘 - "
                          f"조향 최소 분해능 한계, pot={pot})")
                    return pot
        prev_err = err

        # 오차에 비례한 짧은 펄스
        step = min(max(abs(err) * 0.004, 0.03), burst)
        pulse(ser, (sign_up if err > 0 else -sign_up) * pwm, step)

    stop(ser)
    pot = pot_now(ser)
    print(f"    !! 중앙 복귀 실패 (현재 pot={pot}, 목표 {CENTER})")
    return pot


def find_min_pwm(ser):
    """움직이기 시작하는 최소 PWM과, pot을 키우는 PWM 부호를 찾는다."""
    print("[1단계] 조향이 움직이기 시작하는 최소 PWM 탐색")
    print("        중앙 근처를 벗어나지 않도록 방향을 번갈아 가며 짧게 민다.\n")

    sign = +1
    for pwm in SEARCH_STEPS:
        input(f"  [Enter] PWM {pwm} 시도 ... ")
        start, end = push_for(ser, sign * pwm, 0.6)
        moved = end - start
        print(f"      pot {start} -> {end}  이동 {moved:+d}")

        if abs(moved) >= 4:
            sign_up = sign if moved > 0 else -sign
            print(f"\n  => 최소 구동 PWM = {pwm}")
            print(f"  => PWM 양수(+)를 주면 pot이 "
                  f"{'커진다' if sign_up > 0 else '작아진다'}\n")
            return pwm, sign_up

        sign = -sign      # 안 움직였으면 다음엔 반대로 (중앙 이탈 방지)

    raise Aborted(f"PWM {SEARCH_STEPS[-1]}까지 올려도 조향이 안 움직입니다. "
                  "기계적 걸림이나 배선을 확인하세요.")


def measure_travel(ser, min_pwm, sign_up, recenter_pwm):
    """PWM별 이동량을 측정한다. 매번 중앙 복귀 후 측정."""
    print("[2단계] PWM별 이동량 측정 (매 측정 전 중앙 복귀)\n")
    rows = []
    steps = [p for p in SEARCH_STEPS if p >= min_pwm]

    for pwm in steps:
        for sgn, name in ((-1, '음(-)'), (+1, '양(+)')):
            input(f"  [Enter] PWM {pwm:3d} {name} 방향 ... ")
            c = goto_center(ser, recenter_pwm, sign_up)
            start, end = push_for(ser, sgn * pwm, MEASURE_SEC)
            moved = end - start
            print(f"      중앙복귀 {c} | pot {start} -> {end} | "
                  f"{MEASURE_SEC}초 이동 {moved:+d}")
            rows.append((pwm, sgn, start, end, moved))
    print()
    return rows


def measure_limits(ser, pwm, sign_up, recenter_pwm):
    """실제 기계적 한계(pot 최소/최대)를 측정한다."""
    print("[3단계] 기계적 한계 측정 - 더 안 움직일 때까지 민다")
    print("        스토퍼 소리가 나면 Ctrl+C\n")
    limits = {}
    for sgn, name in ((-1, '음(-)'), (+1, '양(+)')):
        input(f"  [Enter] PWM {pwm} {name} 방향 끝까지 ... ")
        goto_center(ser, recenter_pwm, sign_up)
        start, end, reached = push_until_stuck(ser, sgn * pwm)
        tag = '한계 도달' if reached else '시간 초과'
        print(f"      pot {start} -> {end}  ({tag})")
        limits[name] = end
    print()
    goto_center(ser, recenter_pwm, sign_up)
    return limits


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2.5)
    ser.reset_input_buffer()

    rows, limits, min_pwm = [], {}, None
    try:
        pot = pot_now(ser)
        print(f"시작 pot = {pot}   (직진 기준 {CENTER})\n")

        min_pwm, sign_up = find_min_pwm(ser)
        recenter_pwm = min_pwm + RECENTER_MARGIN

        goto_center(ser, recenter_pwm, sign_up)
        rows = measure_travel(ser, min_pwm, sign_up, recenter_pwm)
        limits = measure_limits(ser, min(min_pwm + 60, 200), sign_up, recenter_pwm)

    except KeyboardInterrupt:
        print("\n\n중단됨 - 모터 정지")
    except Aborted as e:
        print(f"\n중단: {e}")
    finally:
        stop(ser)
        ser.close()

    if rows:
        print("=" * 52)
        print(f"{'PWM':>5} {'방향':>5} {'시작':>6} {'끝':>6} {'이동':>7}")
        print("-" * 52)
        for pwm, sgn, s, e, moved in rows:
            print(f"{pwm:5d} {'음' if sgn < 0 else '양':>5} "
                  f"{s:6d} {e:6d} {moved:+7d}")
        print("-" * 52)
    if min_pwm:
        print(f"최소 구동 PWM : {min_pwm}")
    if limits:
        print(f"기계적 한계   : 음(-) {limits.get('음(-)')}  /  "
              f"양(+) {limits.get('양(+)')}")
        print("현재 펌웨어   : Min 429 / Center 510 / Max 632")


if __name__ == '__main__':
    sys.exit(main())
