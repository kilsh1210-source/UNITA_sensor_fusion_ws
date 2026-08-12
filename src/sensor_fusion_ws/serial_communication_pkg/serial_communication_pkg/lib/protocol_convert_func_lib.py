def convert_serial_message(steer_norm, rear_pwm):
    """아두이노(autonomous_mega/src/comm.ino)가 받는 형식으로 만든다.

    형식: "C,<steer_norm>,<rear_pwm>\n"  (필드 2개, 쉼표 2개)
      steer_norm : 조향 목표. -1.0(좌) ~ 0.0(중립) ~ +1.0(우) 정규화 값.
                   펌웨어가 normalizedToSteeringPosition()으로 포텐셔미터
                   목표값(steeringPotMin..Max)으로 바꾼다. PWM이 아니다.
      rear_pwm   : 구동 모터 PWM. -255 ~ 255.

    주의: comm.ino의 handleLine()은 쉼표가 3개 이상이면
          (idx3 != -1) 줄 전체를 버린다. 예전처럼 u_cmd를 뒤에 붙이면
          명령이 통째로 무시되고 워치독(DRIVE_WD_MS)이 모터를 0으로 만든다.
    """
    return f"C,{steer_norm:.4f},{int(rear_pwm)}\n"
