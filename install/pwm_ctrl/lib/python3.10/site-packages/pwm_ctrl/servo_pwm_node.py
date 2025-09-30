#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO

# ===== 기본 파라미터 =====F
# PWM 허용 핀(예시): BOARD 33(좌), 35(우) 권장
DEFAULT_LEFT_PIN   = 33
DEFAULT_RIGHT_PIN  = 32
DEFAULT_F_HZ       = 50.0
DEFAULT_TOPIC_CMD  = "/atoz/cmd_vel"         # Twist: linear.x, angular.z
DEFAULT_NEUTRAL    = 1500               # µs
DEFAULT_WD_MS      = 500
DEFAULT_ARMING_S   = 2.0

# 입력 스케일 선택:
# input_mode = "us"  → linear.x, angular.z를 µs로 직접 사용 (예: 1500, ±200)
# input_mode = "norm"→ linear.x∈[-1,1], angular.z∈[-1,1] 로 보내고 아래 스케일로 변환
DEFAULT_INPUT_MODE = "norm"             # "us" 또는 "norm"
DEFAULT_THR_MIN    = 1000               # us (정규화 모드에서 -1.0이 맵핑될 값)
DEFAULT_THR_MAX    = 2000               # us (정규화 모드에서 +1.0이 맵핑될 값)
DEFAULT_STEER_MAX  = 300                # us (정규화 모드에서 |angular.z|=1.0 → ±300us)

# 채널별 보정
DEFAULT_INVERT_L   = False
DEFAULT_INVERT_R   = True
DEFAULT_MIN_US     = 900
DEFAULT_MAX_US     = 2100
DEFAULT_DBAND_US   = 20

class DiffDrivePwmNode(Node):
    def __init__(self) -> None:
        super().__init__("diff_drive_pwm_node")

        # ----- 파라미터 선언 -----
        self.declare_parameter("left_pin",    DEFAULT_LEFT_PIN)
        self.declare_parameter("right_pin",   DEFAULT_RIGHT_PIN)
        self.declare_parameter("freq_hz",     DEFAULT_F_HZ)
        self.declare_parameter("cmd_topic",   DEFAULT_TOPIC_CMD)
        self.declare_parameter("neutral_us",  DEFAULT_NEUTRAL)
        self.declare_parameter("watchdog_ms", DEFAULT_WD_MS)
        self.declare_parameter("arming_sec",  DEFAULT_ARMING_S)

        self.declare_parameter("input_mode",  DEFAULT_INPUT_MODE)  # "us"|"norm"
        self.declare_parameter("thr_min_us",  DEFAULT_THR_MIN)
        self.declare_parameter("thr_max_us",  DEFAULT_THR_MAX)
        self.declare_parameter("steer_max_us",DEFAULT_STEER_MAX)

        self.declare_parameter("invert_left",  DEFAULT_INVERT_L)
        self.declare_parameter("invert_right", DEFAULT_INVERT_R)
        self.declare_parameter("min_us",       DEFAULT_MIN_US)
        self.declare_parameter("max_us",       DEFAULT_MAX_US)
        self.declare_parameter("deadband_us",  DEFAULT_DBAND_US)

        # ----- 파라미터 로드 -----
        self.left_pin   = int(self.get_parameter("left_pin").value)
        self.right_pin  = int(self.get_parameter("right_pin").value)
        self.f_hz       = float(self.get_parameter("freq_hz").value)
        self.cmd_topic  = str(self.get_parameter("cmd_topic").value)
        self.neutral    = int(self.get_parameter("neutral_us").value)
        self.wd_ms      = int(self.get_parameter("watchdog_ms").value)
        self.arming_sec = float(self.get_parameter("arming_sec").value)

        self.input_mode = str(self.get_parameter("input_mode").value).lower()
        self.thr_min    = int(self.get_parameter("thr_min_us").value)
        self.thr_max    = int(self.get_parameter("thr_max_us").value)
        self.steer_max  = int(self.get_parameter("steer_max_us").value)

        self.invert_l   = bool(self.get_parameter("invert_left").value)
        self.invert_r   = bool(self.get_parameter("invert_right").value)
        self.min_us     = int(self.get_parameter("min_us").value)
        self.max_us     = int(self.get_parameter("max_us").value)
        self.deadband   = int(self.get_parameter("deadband_us").value)

        # ----- GPIO 초기화 -----
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.left_pin,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_pin, GPIO.OUT, initial=GPIO.LOW)

        self.pwm_left  = GPIO.PWM(self.left_pin,  self.f_hz)
        self.pwm_right = GPIO.PWM(self.right_pin, self.f_hz)

        # 중립 듀티로 시작 (아밍)
        nd = self._us_to_duty(self.neutral)
        self.pwm_left.start(nd)
        self.pwm_right.start(nd)

        now = self.get_clock().now()
        self.arming_until = now + Duration(seconds=self.arming_sec)
        self.last_rcv     = now

        # 하나의 토픽만 구독 (Twist)
        self.sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)

        # 워치독
        self.timer = self.create_timer(self.wd_ms / 1000.0, self.watchdog)

        self.get_logger().info(
            f"[diff_drive_pwm_node] L={self.left_pin}, R={self.right_pin}, "
            f"freq={self.f_hz}Hz, neutral={self.neutral}us, input='{self.input_mode}', "
            f"thr_min={self.thr_min}, thr_max={self.thr_max}, steer_max={self.steer_max}, "
            f"invert(L,R)=({self.invert_l},{self.invert_r}), range=[{self.min_us},{self.max_us}]"
        )

    # ===== 유틸 =====
    def _us_to_duty(self, us: int) -> float:
        period_us = 1_000_000.0 / self.f_hz
        duty = (float(us) / period_us) * 100.0
        return max(0.0, min(100.0, duty))

    def _apply_channel_map(self, us: int, invert: bool) -> int:
        if invert:
            us = 2 * self.neutral - us
        if abs(us - self.neutral) <= self.deadband:
            us = self.neutral
        if us < self.min_us:
            us = self.min_us
        elif us > self.max_us:
            us = self.max_us
        return us

    # ===== 메인 콜백 =====
    def on_cmd(self, msg: Twist) -> None:
        now = self.get_clock().now()
        if now < self.arming_until:
            return

        # 입력 해석
        if self.input_mode == "us":
            throttle_us = int(msg.linear.x)       # 절대 펄스폭
            steer_delta = int(msg.angular.z)      # 중립 기준 ±us
        else:  # "norm": -1.0~+1.0
            # throttle: [-1..+1] → [thr_min..thr_max]
            t = max(-1.0, min(1.0, msg.linear.x))
            throttle_us = int((t + 1.0) * 0.5 * (self.thr_max - self.thr_min) + self.thr_min)
            # steer: [-1..+1] → [-steer_max..+steer_max]
            s = max(-1.0, min(1.0, msg.angular.z))
            steer_delta = int(s * self.steer_max)

        # 믹싱 (좌:+, 우:-)
        left_us  = throttle_us + steer_delta
        right_us = throttle_us - steer_delta

        # 채널별 보정/클램프 → 출력
        l = self._apply_channel_map(left_us,  self.invert_l)
        r = self._apply_channel_map(right_us, self.invert_r)

        self.pwm_left.ChangeDutyCycle(self._us_to_duty(l))
        self.pwm_right.ChangeDutyCycle(self._us_to_duty(r))

        self.last_rcv = now

    def watchdog(self) -> None:
        if (self.get_clock().now() - self.last_rcv).nanoseconds > self.wd_ms * 1_000_000:
            nd = self._us_to_duty(self.neutral)
            self.pwm_left.ChangeDutyCycle(nd)
            self.pwm_right.ChangeDutyCycle(nd)

    def destroy_node(self) -> None:
        try:
            self.pwm_left.stop()
            self.pwm_right.stop()
        finally:
            GPIO.cleanup()
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DiffDrivePwmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

