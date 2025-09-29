#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
import Jetson.GPIO as GPIO
import time

# 기본 파라미터(ros2 param으로 바꿀 수 있음)
DEFAULT_BOARD_PIN = 32     # BOARD 모드의 물리 핀 번호
DEFAULT_F_HZ      = 50.0   # 50 Hz (서보/ESC)
DEFAULT_TOPIC     = '/atoz/base_link'  # 단위: 마이크로초(µs), 예: 1000~2000

class ServoPwmNode(Node):
    def __init__(self):
        super().__init__('servo_pwm_node')

        # 파라미터 선언
        self.declare_parameter('board_pin', DEFAULT_BOARD_PIN)
        self.declare_parameter('freq_hz',   DEFAULT_F_HZ)
        self.declare_parameter('topic',     DEFAULT_TOPIC)
        self.declare_parameter('neutral_us', 1500)  # 워치독 중립값
        self.declare_parameter('watchdog_ms', 500)  # ms

        self.pin       = int(self.get_parameter('board_pin').value)
        self.f_hz      = float(self.get_parameter('freq_hz').value)
        self.topic     = str(self.get_parameter('topic').value)
        self.neutral   = int(self.get_parameter('neutral_us').value)
        self.wd_ms     = int(self.get_parameter('watchdog_ms').value)

        # Jetson.GPIO 설정
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)  # 물리 핀 번호 체계
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

        self.pwm = GPIO.PWM(self.pin, self.f_hz)
        self.pwm.start(0.0)

        # 토픽 구독 시작
        self.sub = self.create_subscription(UInt16, self.topic, self.on_msg, 10)

        # 워치독: 일정 시간 입력 없으면 중립으로
        self.last_rcv = self.get_clock().now()
        self.timer    = self.create_timer(self.wd_ms / 1000.0, self.watchdog)

        self.get_logger().info(
            f'[{self.get_name()}] pin=BOARD {self.pin}, freq={self.f_hz}Hz, '
            f'topic="{self.topic}", neutral={self.neutral}us, watchdog={self.wd_ms}ms'
        )

    def set_us(self, us: int):
        # 20ms(=1/f) 기준 듀티 변환 (서보/ESC는 50Hz가 보통이므로 20ms 가정)
        # 만약 freq를 바꾸면 duty 계산을 (us / (1000000/freq)) * 100 으로 일반화
        period_us = int(1_000_000 / self.f_hz)
        duty = (us / period_us) * 100.0
        duty = max(0.0, min(100.0, duty))
        self.pwm.ChangeDutyCycle(duty)

    def on_msg(self, msg: UInt16):
        us = int(msg.data)
        self.set_us(us)
        self.last_rcv = self.get_clock().now()
        self.get_logger().debug(f'set {us}us')

    def watchdog(self):
        # wd_ms 이상 토픽 입력 없으면 중립값으로 유지
        if (self.get_clock().now() - self.last_rcv).nanoseconds > self.wd_ms * 1_000_000:
            self.set_us(self.neutral)

    def destroy_node(self):
        try:
            self.pwm.stop()
            GPIO.cleanup()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = ServoPwmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

