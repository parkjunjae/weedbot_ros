#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
from enum import Enum

# ===== 하드웨어 매핑 =====
DEFAULT_THROTTLE_PIN = 32   # ESC 스로틀 (CH2, 3핀 묶음)  -> BOARD numbering
DEFAULT_STEER_PIN    = 33   # 스티어/회전 채널 (CH4, 단독) -> BOARD numbering

# ===== 기본 파라미터 =====
DEFAULT_F_HZ         = 50.0
DEFAULT_TOPIC_CMD    = "/atoz/cmd_vel"
DEFAULT_NEUTRAL      = 1500
DEFAULT_WD_MS        = 500
DEFAULT_ARMING_S     = 6.0

DEFAULT_STEER_MAX    = 300
DEFAULT_DBAND_US     = 30

# ==== ESC 스펙 기반 맵 ====
# 정규화 입력 t ∈ [-1, 1]을 ESC 펄스로 변환할 때 쓰는 기준점
DEFAULT_FWD_NEAR_US  = 1400   # t=0 에서 전진쪽 기준
DEFAULT_FWD_FAR_US   = 900    # t=+1 최댓전진
DEFAULT_REV_NEAR_US  = 1700   # t=0 에서 후진쪽 기준
DEFAULT_REV_FAR_US   = 2100   # t=-1 최댓후진

# 옵션
DEFAULT_REVERSE_GATE_MS = 0          # 전/후진 급변 시 중립 유지 시간(ms), 0이면 비활성
DEFAULT_LOG_LEVEL_DEBUG = True
DEFAULT_ARMING_STYLE    = "min"       # "neutral" | "min" (전원 직후 최저로 아밍)
DEFAULT_FORCE_THR_US    = 0           # 0=off, >0 이면 스로틀 강제 펄스(테스트용)

# 스티어 PWM 비활성화(스로틀만 구동해서 지터 최소화)
DEFAULT_ENABLE_STEER         = True

# 새 옵션: 회전-only 모드 / 스티어 채널 타입
DEFAULT_ROTATE_ON_STEER_ONLY = True   # 선속≈0 & 각속!=0 인 경우 33번만 PWM
DEFAULT_STEER_CONTROLS_MOTOR = True   # CH4가 모터/ESC면 True, 서보면 False

class RevGateState(Enum):
    IDLE = 0
    GATE = 1

class ThrottleSteerNode(Node):
    def __init__(self):
        super().__init__("throttle_steer_node")

        # 파라미터 선언
        self.declare_parameter("throttle_pin", DEFAULT_THROTTLE_PIN)
        self.declare_parameter("steer_pin",    DEFAULT_STEER_PIN)
        self.declare_parameter("freq_hz",      DEFAULT_F_HZ)
        self.declare_parameter("cmd_topic",    DEFAULT_TOPIC_CMD)
        self.declare_parameter("neutral_us",   DEFAULT_NEUTRAL)
        self.declare_parameter("watchdog_ms",  DEFAULT_WD_MS)
        self.declare_parameter("arming_sec",   DEFAULT_ARMING_S)

        self.declare_parameter("steer_max_us", DEFAULT_STEER_MAX)
        self.declare_parameter("deadband_us",  DEFAULT_DBAND_US)

        self.declare_parameter("fwd_near_us",  DEFAULT_FWD_NEAR_US)
        self.declare_parameter("fwd_far_us",   DEFAULT_FWD_FAR_US)
        self.declare_parameter("rev_near_us",  DEFAULT_REV_NEAR_US)
        self.declare_parameter("rev_far_us",   DEFAULT_REV_FAR_US)

        self.declare_parameter("reverse_gate_ms", DEFAULT_REVERSE_GATE_MS)
        self.declare_parameter("debug_log",       DEFAULT_LOG_LEVEL_DEBUG)
        self.declare_parameter("arming_style",    DEFAULT_ARMING_STYLE)
        self.declare_parameter("force_thr_us",    DEFAULT_FORCE_THR_US)

        self.declare_parameter("enable_steer",          DEFAULT_ENABLE_STEER)
        self.declare_parameter("rotate_on_steer_only",  DEFAULT_ROTATE_ON_STEER_ONLY)
        self.declare_parameter("steer_controls_motor",  DEFAULT_STEER_CONTROLS_MOTOR)

        # 파라미터 로드
        self.thr_pin   = int(self.get_parameter("throttle_pin").value)
        self.steer_pin = int(self.get_parameter("steer_pin").value)
        self.f_hz      = float(self.get_parameter("freq_hz").value)
        self.topic     = str(self.get_parameter("cmd_topic").value)
        self.neutral   = int(self.get_parameter("neutral_us").value)
        self.wd_ms     = int(self.get_parameter("watchdog_ms").value)
        self.arm_s     = float(self.get_parameter("arming_sec").value)

        self.steer_max = int(self.get_parameter("steer_max_us").value)
        self.deadband  = int(self.get_parameter("deadband_us").value)

        self.fwd_near  = int(self.get_parameter("fwd_near_us").value)
        self.fwd_far   = int(self.get_parameter("fwd_far_us").value)
        self.rev_near  = int(self.get_parameter("rev_near_us").value)
        self.rev_far   = int(self.get_parameter("rev_far_us").value)

        self.reverse_gate_ms = int(self.get_parameter("reverse_gate_ms").value)
        self.debug_log       = bool(self.get_parameter("debug_log").value)
        self.force_thr_us    = int(self.get_parameter("force_thr_us").value)

        self.arming_style = str(self.get_parameter("arming_style").value).strip().lower()
        if self.arming_style not in ("neutral", "min"):
            self.get_logger().warn(f"Unknown arming_style '{self.arming_style}', fallback to 'neutral'")
            self.arming_style = "neutral"

        self.enable_steer        = bool(self.get_parameter("enable_steer").value)
        self.rotate_on_steer_only = bool(self.get_parameter("rotate_on_steer_only").value)
        self.steer_controls_motor = bool(self.get_parameter("steer_controls_motor").value)

        # 내부 상태
        self.last_thr_us = self.neutral
        self.rev_state   = RevGateState.IDLE
        self.rev_gate_until = None

        # GPIO 초기화
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.thr_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.enable_steer:
            GPIO.setup(self.steer_pin, GPIO.OUT, initial=GPIO.LOW)

        # PWM 객체
        self.pwm_thr = GPIO.PWM(self.thr_pin, self.f_hz)
        self.pwm_steer = None
        if self.enable_steer:
            self.pwm_steer = GPIO.PWM(self.steer_pin, self.f_hz)

        # 아밍 펄스
        self.arming_us = self.neutral if self.arming_style == "neutral" else min(self.fwd_far, self.neutral)
        self.pwm_thr.start(self._us_to_duty(self.arming_us))
        if self.pwm_steer:
            self.pwm_steer.start(self._us_to_duty(self.neutral))

        now = self.get_clock().now()
        self.arming_until = now + Duration(seconds=self.arm_s)
        self.last_rcv     = now

        # ROS I/O
        self.sub   = self.create_subscription(Twist, self.topic, self.on_cmd, 10)
        self.timer = self.create_timer(self.wd_ms / 1000.0, self.watchdog)

        self.get_logger().info(
            f"[throttle_steer_node] Throttle=BOARD{self.thr_pin} CH2, "
            f"Steer={'ENABLED BOARD'+str(self.steer_pin) if self.enable_steer else 'DISABLED'}, "
            f"freq={self.f_hz}Hz, neutral={self.neutral}us, "
            f"FWD:{self.fwd_near}->{self.fwd_far}us, REV:{self.rev_near}->{self.rev_far}us, "
            f"arming={self.arm_s}s({self.arming_style}:{self.arming_us}us), wd={self.wd_ms}ms, "
            f"rotate_only={self.rotate_on_steer_only}, steer_as_motor={self.steer_controls_motor}, "
            f"force_thr_us={self.force_thr_us}"
        )

    # ===== 유틸 =====
    def _us_to_duty(self, us: int) -> float:
        period_us = 1_000_000.0 / self.f_hz
        return max(0.0, min(100.0, (us / period_us) * 100.0))

    def _clamp_us(self, us: int) -> int:
        lo = min(self.fwd_far, self.neutral, self.rev_near)
        hi = max(self.rev_far, self.neutral, self.fwd_near)
        return max(lo, min(hi, us))

    def _map_throttle_piecewise(self, t: float) -> int:
        """[-1..1] -> ESC 펄스(전/후진 분할 맵)"""
        if abs(t) < max(1e-3, self.deadband / 1000.0):
            return self.neutral
        if t > 0.0:
            us = int(self.fwd_near + (self.fwd_far - self.fwd_near) * min(t, 1.0))   # 1400→900
        else:
            us = int(self.rev_near + (self.rev_far - self.rev_near) * min(-t, 1.0))  # 1700→2100
        return self._clamp_us(us)

    def _map_esc_like(self, v_norm: float) -> int:
        """스티어 채널을 ESC처럼 쓸 때의 매핑 [-1..1] -> ESC 펄스"""
        v = max(-1.0, min(1.0, v_norm))
        if abs(v) < max(1e-3, self.deadband / 1000.0):
            return self.neutral
        if v > 0.0:
            us = int(self.fwd_near + (self.fwd_far - self.fwd_near) * v)
        else:
            us = int(self.rev_near + (self.rev_far - self.rev_near) * (-v))
        return self._clamp_us(us)

    def _map_steer_servo(self, s: float) -> int:
        """서보(각도)로 쓸 때의 매핑: 1500±steer_max"""
        if not self.pwm_steer:
            return self.neutral
        lo = self.neutral - self.steer_max
        hi = self.neutral + self.steer_max
        if abs(s) < 1e-6:
            return self.neutral
        us = self.neutral + int(max(-1.0, min(1.0, s)) * self.steer_max)
        return max(lo, min(hi, us))

    def _apply_pwm(self, thr_us: int, steer_us: int):
        self.pwm_thr.ChangeDutyCycle(self._us_to_duty(thr_us))
        if self.pwm_steer:
            self.pwm_steer.ChangeDutyCycle(self._us_to_duty(steer_us))

    # ===== 콜백 =====
    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()

        # 아밍 중에는 스로틀만 유지
        if now < self.arming_until:
            self._apply_pwm(self.arming_us, self.neutral)
            return

        # 입력 정규화 (Twist: linear.x, angular.z)
        t = max(-1.0, min(1.0, msg.linear.x))
        s = max(-1.0, min(1.0, msg.angular.z))

        # 회전-only 조건: 선속 거의 0, 각속 존재
        t_dead = max(1e-3, self.deadband / 1000.0)
        rotate_only = self.rotate_on_steer_only and (abs(t) < t_dead) and (abs(s) >= t_dead)

        if rotate_only:
            # 1) 스로틀(CH2, BOARD32)은 항상 중립
            target_thr_us = self.neutral

            # 2) 스티어(CH4, BOARD33)만 사용
            if self.pwm_steer:
                if self.steer_controls_motor:
                    # CH4가 모터/ESC 라면 ESC 맵핑 사용
                    target_steer_us = self._map_esc_like(s)
                else:
                    # CH4가 서보 라면 1500±steer_max 각도 제어
                    target_steer_us = self._map_steer_servo(s)
            else:
                target_steer_us = self.neutral  # 안전 중립
        else:
            # 직진/후진/곡선: CH2만 사용, CH4는 중립으로 잠금(지터 최소화)
            target_thr_us   = self._map_throttle_piecewise(t) if self.force_thr_us == 0 else self._clamp_us(self.force_thr_us)
            target_steer_us = self.neutral

        # 역진 게이트: 스로틀 변경이 있을 때만 관여 (회전-only에는 적용 X)
        if self.reverse_gate_ms > 0 and self.force_thr_us == 0 and not rotate_only:
            cur_fwd = self.last_thr_us < (self.neutral - self.deadband)
            new_rev = target_thr_us > (self.neutral + self.deadband)
            if self.rev_state == RevGateState.IDLE and cur_fwd and new_rev:
                self.rev_state = RevGateState.GATE
                self.rev_gate_until = now + Duration(seconds=self.reverse_gate_ms / 1000.0)
                self._apply_pwm(self.neutral, target_steer_us)
                self.last_rcv = now
                self.last_thr_us = self.neutral
                if self.debug_log:
                    self.get_logger().info(f"[rev-gate] neutral {self.reverse_gate_ms}ms")
                return
            if self.rev_state == RevGateState.GATE:
                if now < self.rev_gate_until:
                    self._apply_pwm(self.neutral, target_steer_us)
                    self.last_rcv = now
                    self.last_thr_us = self.neutral
                    return
                self.rev_state = RevGateState.IDLE

        # 적용
        self._apply_pwm(target_thr_us, target_steer_us)
        self.last_rcv = now
        self.last_thr_us = target_thr_us

        if self.debug_log:
            self.get_logger().info(
                f"{'ROTATE-ONLY' if rotate_only else 'DRIVE'} "
                f"thr_us={target_thr_us}, steer_us={target_steer_us}, t={t:.2f}, s={s:.2f}"
            )

    def watchdog(self):
        if (self.get_clock().now() - self.last_rcv).nanoseconds > self.wd_ms * 1_000_000:
            nd = self._us_to_duty(self.neutral)
            self.pwm_thr.ChangeDutyCycle(nd)
            if self.pwm_steer:
                self.pwm_steer.ChangeDutyCycle(nd)
            self.rev_state = RevGateState.IDLE

    def destroy_node(self):
        try:
            self.pwm_thr.stop()
            if self.pwm_steer:
                self.pwm_steer.stop()
        finally:
            GPIO.cleanup()
            super().destroy_node()

def main():
    rclpy.init()
    node = ThrottleSteerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
