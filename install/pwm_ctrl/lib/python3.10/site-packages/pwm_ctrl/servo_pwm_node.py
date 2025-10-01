#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
from enum import Enum




# ===== 하드웨어 매핑 (BOARD 번호) =====
DEFAULT_THROTTLE_PIN = 32   # CH2
DEFAULT_STEER_PIN    = 33   # CH4

# ===== 기본 파라미터 =====
DEFAULT_F_HZ         = 50.0
DEFAULT_TOPIC_CMD    = "/atoz/cmd_vel"
DEFAULT_NEUTRAL      = 1500
DEFAULT_WD_MS        = 500
DEFAULT_ARMING_S     = 6.0

# ESC / 서보 범위 등
DEFAULT_STEER_MAX    = 300
DEFAULT_DBAND_US     = 30

# ==== ESC 맵 (정규화 입력 [-1,1]) ====
DEFAULT_FWD_NEAR_US  = 1400   # t=0 → 1400
DEFAULT_FWD_FAR_US   = 900    # t=+1 → 900
DEFAULT_REV_NEAR_US  = 1700   # t=0 → 1700
DEFAULT_REV_FAR_US   = 2100   # t=-1 → 2100

# 옵션
DEFAULT_REVERSE_GATE_MS = 0
DEFAULT_LOG_LEVEL_DEBUG = True
DEFAULT_ARMING_STYLE    = "min"   # "neutral"|"min"
DEFAULT_FORCE_THR_US    = 0       # 0=off

# 새 옵션들
DEFAULT_ENABLE_STEER         = True
DEFAULT_MODE                 = "simple_split"  # "simple_split" | "nav_tank_mix"
DEFAULT_TANK_K               = 0.5             # nav_tank_mix에서 회전 비중 k(바퀴폭/2 등)
DEFAULT_INVERT_TURN_DIR      = False           # 회전 부호 뒤집기(좌/우 반전 교정용)

class RevGateState(Enum):
    IDLE = 0
    GATE = 1

class ThrottleSteerNode(Node):
    def __init__(self):
        super().__init__("throttle_steer_node")

        # ---- 파라미터 선언 ----
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

        self.declare_parameter("enable_steer",        DEFAULT_ENABLE_STEER)
        self.declare_parameter("mode",                DEFAULT_MODE)          # "simple_split" | "nav_tank_mix"
        self.declare_parameter("tank_k",              DEFAULT_TANK_K)
        self.declare_parameter("invert_turn_dir",     DEFAULT_INVERT_TURN_DIR)

        # ---- 파라미터 로드 ----
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

        self.enable_steer  = bool(self.get_parameter("enable_steer").value)
        self.mode          = str(self.get_parameter("mode").value).strip().lower()
        self.tank_k        = float(self.get_parameter("tank_k").value)
        self.invert_turn   = bool(self.get_parameter("invert_turn_dir").value)

        # 내부 상태
        self.last_thr_us = self.neutral
        self.rev_state   = RevGateState.IDLE
        self.rev_gate_until = None

        # GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.thr_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.enable_steer:
            GPIO.setup(self.steer_pin, GPIO.OUT, initial=GPIO.LOW)

        # PWM
        self.pwm_thr = GPIO.PWM(self.thr_pin, self.f_hz)
        self.pwm_steer = None
        if self.enable_steer:
            self.pwm_steer = GPIO.PWM(self.steer_pin, self.f_hz)

        # 아밍
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
            f"[throttle_steer_node] CH2=BOARD{self.thr_pin}, CH4=BOARD{self.steer_pin if self.enable_steer else 'DISABLED'}, "
            f"freq={self.f_hz}Hz, neutral={self.neutral}us, mode={self.mode}, k={self.tank_k}, invert_turn={self.invert_turn}"
        )

    # ===== 유틸 =====
    def _us_to_duty(self, us: int) -> float:
        period_us = 1_000_000.0 / self.f_hz
        return max(0.0, min(100.0, (us / period_us) * 100.0))

    def _clamp_us(self, us: int) -> int:
        lo = min(self.fwd_far, self.neutral, self.rev_near)
        hi = max(self.rev_far, self.neutral, self.fwd_near)
        return max(lo, min(hi, us))

    def _map_esc(self, v: float) -> int:
        """정규화 [-1..1] -> ESC 펄스폭"""
        v = max(-1.0, min(1.0, v))
        if abs(v) < max(1e-3, self.deadband / 1000.0):
            return self.neutral
        if v > 0.0:
            us = int(self.fwd_near + (self.fwd_far - self.fwd_near) * v)     # 1400→900
        else:
            us = int(self.rev_near + (self.rev_far - self.rev_near) * (-v))  # 1700→2100
        return self._clamp_us(us)

    def _apply_pwm(self, ch2_us: int, ch4_us: int):
        self.pwm_thr.ChangeDutyCycle(self._us_to_duty(ch2_us))
        if self.pwm_steer:
            self.pwm_steer.ChangeDutyCycle(self._us_to_duty(ch4_us))

    # ===== 콜백 =====
    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()

        # 아밍 동안
        if now < self.arming_until:
            self._apply_pwm(self.arming_us, self.neutral)
            return

        # 입력 정규화
        t = max(-1.0, min(1.0, msg.linear.x))
        s = max(-1.0, min(1.0, msg.angular.z))
        if self.invert_turn:
            s = -s  # 회전 방향 뒤집기

        if self.mode == "nav_tank_mix":
            # ---- 자율주행용 풀 믹싱 (오른쪽 모터 물리 반전 가정) ----
            # Left_cmd  = v - k*w
            # Right_cmd = -(v + k*w)   # 오른쪽이 뒤집혀 있으므로 부호 반전
            left_cmd  = t - self.tank_k * s
            right_cmd = -(t + self.tank_k * s)
            ch2_us = self._map_esc(left_cmd)    # CH2(32) = Left
            ch4_us = self._map_esc(right_cmd)   # CH4(33) = Right
        else:
            # ---- simple_split 모드 ----
            t_dead = max(1e-3, self.deadband / 1000.0)
            rotate_only = (abs(t) < t_dead) and (abs(s) >= t_dead)

            if rotate_only:
                # 제자리 회전: 좌/우 같은 방향
                ch2_us = self._map_esc(s)
                ch4_us = self._map_esc(s)
            else:
                # 직진/후진: 좌/우 서로 반대 방향
                # CH2 = t, CH4 = -t
                ch2_us = self._map_esc(t)
                ch4_us = self._map_esc(-t)

        # 역진 게이트(옵션) — 단, force_thr_us 사용 안 하고 simple_split에서 전후진일 때만 사용
        if self.reverse_gate_ms > 0 and self.force_thr_us == 0 and self.mode != "nav_tank_mix":
            t_dead = max(1e-3, self.deadband / 1000.0)
            rotate_only = (abs(t) < t_dead) and (abs(s) >= t_dead)
            if not rotate_only:
                cur_fwd = self.last_thr_us < (self.neutral - self.deadband)
                new_rev = ch2_us > (self.neutral + self.deadband)  # CH2 기준
                if self.rev_state == RevGateState.IDLE and cur_fwd and new_rev:
                    self.rev_state = RevGateState.GATE
                    self.rev_gate_until = now + Duration(seconds=self.reverse_gate_ms / 1000.0)
                    self._apply_pwm(self.neutral, self.neutral)
                    self.last_rcv   = now
                    self.last_thr_us = self.neutral
                    if self.debug_log:
                        self.get_logger().info(f"[rev-gate] neutral {self.reverse_gate_ms}ms")
                    return
                if self.rev_state == RevGateState.GATE:
                    if now < self.rev_gate_until:
                        self._apply_pwm(self.neutral, self.neutral)
                        self.last_rcv   = now
                        self.last_thr_us = self.neutral
                        return
                    self.rev_state = RevGateState.IDLE

        # 적용
        self._apply_pwm(ch2_us, ch4_us)
        self.last_rcv = now
        self.last_thr_us = ch2_us

        if self.debug_log:
            self.get_logger().info(
                f"{self.mode.upper()} ch2={ch2_us} ch4={ch4_us}  t={t:.2f} s={s:.2f}"
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
