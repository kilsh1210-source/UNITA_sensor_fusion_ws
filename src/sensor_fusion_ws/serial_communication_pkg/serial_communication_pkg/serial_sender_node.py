import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT = '/dev/ttyACM0'
BAUDRATE = 115200
#----------------------------------------------

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')

    self.declare_parameter('sub_topic', sub_topic)
    # 아두이노 포트/속도. launch 인자(serial_sender_port)로도 바꿀 수 있음
    self.declare_parameter('port', PORT)
    self.declare_parameter('baudrate', BAUDRATE)
    # steering(-max_steer_cmd..+max_steer_cmd)을 아두이노용 -1.0..+1.0으로 정규화
    self.declare_parameter('max_steer_cmd', 9.0)
    # 미사용. 아두이노가 조향을 PWM이 아닌 정규화 값으로 받으므로 스케일이 필요 없다.
    # (params.yaml 호환을 위해 선언만 남겨둠)
    self.declare_parameter('steering_pwm_max', 150)
    # 조향 모터 배선 방향이 반대일 경우 True로 뒤집기
    self.declare_parameter('steer_invert', False)

    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.port = self.get_parameter('port').get_parameter_value().string_value
    self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
    self.max_steer_cmd = self.get_parameter('max_steer_cmd').get_parameter_value().double_value
    self.steering_pwm_max = self.get_parameter('steering_pwm_max').get_parameter_value().integer_value
    self.steer_invert = self.get_parameter('steer_invert').get_parameter_value().bool_value

    # 포트를 노드 생성 시점에 연다 (예전에는 import 시점에 열려서, 포트 파라미터가
    # 무시되고 아두이노가 안 꽂혀 있으면 launch 전체가 죽었다)
    self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
    # 아두이노 자동 리셋 후 부트로더가 안정화될 시간을 줌 (너무 빨리 쓰면 부트로더가 멈출 수 있음)
    time.sleep(2)
    self.ser.reset_input_buffer()
    self.get_logger().info(f"serial open: {self.port} @ {self.baudrate}")

    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             depth=1)

    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)

  def data_callback(self, msg):
    steering = msg.steering
    left_speed = msg.left_speed
    right_speed = msg.right_speed

    # 아두이노는 조향을 PWM이 아니라 -1.0~+1.0 정규화 값으로 받는다.
    # (comm.ino -> normalizedToSteeringPosition). steering_pwm_max로 스케일하면
    # 항상 ±1로 constrain 돼서 풀조향만 나온다.
    steer_norm = steering / self.max_steer_cmd
    if self.steer_invert:
        steer_norm = -steer_norm
    steer_norm = max(-1.0, min(1.0, steer_norm))

    rear_pwm = int(max(-255, min(255, round((left_speed + right_speed) / 2))))

    serial_msg = PCFL.convert_serial_message(steer_norm, rear_pwm)
    self.ser.write(serial_msg.encode())

    # 아두이노가 초음파 값을 계속 시리얼로 되돌려 보내는데, 여기서 안 읽어주면
    # 아두이노 쪽 송신 버퍼가 차서 Serial.print()가 블로킹되고 loop()가 멈춰
    # 결국 워치독이 발동해 모터가 꺼진다. 매번 들어온 만큼 비워준다.
    if self.ser.in_waiting:
        self.ser.read(self.ser.in_waiting)

  def stop_motor(self):
    """종료할 때 정지 명령을 보내고 포트를 닫는다."""
    try:
      if self.ser.is_open:
        self.ser.write(PCFL.convert_serial_message(0, 0).encode())
        self.ser.close()
        print('closed')
    except Exception as e:
      print(f'serial close error: {e}')

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)

  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")

  finally:
    node.stop_motor()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
