#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
from enum import Enum

# ===== 하드웨어 매핑 (BOARD 번호) =====
DEFAULT_THROTTLE_PIN = 32   # CH2: 양쪽 스로틀(출력 크기)
DEFAULT_STEER_PIN    = 33   # CH4: 방향(회전)

# ===== 기본 파라미터 =====
DEFAULT_F_HZ         = 50.0
DEFAULT_TOPIC_CMD    = "/atoz/cmd_vel"
DEFAULT_NEUTRAL      = 1500
DEFAULT_WD_MS        = 500
DEFAULT_ARMING_S     = 2.0

# ESC / 서보 범위 등
DEFAULT_STEER_MAX    = 300
DEFAULT_DBAND_US     = 30

# ==== ESC 맵 (정규화 입력 [-1,1]) ====
# 대칭 맵(권장): 중립 1500, 전진/후진 끝값 1000/2000
DEFAULT_FWD_NEAR_US  = 1500   # t=0 → 1500
DEFAULT_FWD_FAR_US   = 1000   # t=+1 → 1000
DEFAULT_REV_NEAR_US  = 1500   # t=0 → 1500
DEFAULT_REV_FAR_US   = 2000   # t=-1 → 2000

# 로깅/아밍/강제
DEFAULT_LOG_LEVEL_DEBUG = True
DEFAULT_ARMING_STYLE    = "neutral"   # "neutral"|"min"
DEFAULT_FORCE_THR_US    = 0           # 0=off

# 동작 모드 (필요시 확장용. 현재는 simple_split 중심)
DEFAULT_MODE            = "simple_split"   # "simple_split" | "nav_tank_mix"
DEFAULT_TANK_K          = 0.5
DEFAULT_INVERT_TURN_DIR = False            # 회전 방향 반전 플래그

# 채널 반전/스왑(기본 꺼둠. 현재 하드웨어 가정상 보통 필요 없음)
DEFAULT_INVERT_CH2    = False
DEFAULT_INVERT_CH4    = False
DEFAULT_SWAP_CHANNELS = False

# 좌/우 밸런스 보정(스케일/트림) — 필요시만 사용
DEFAULT_CH2_SCALE   = 1.0
DEFAULT_CH4_SCALE   = 1.0
DEFAULT_CH2_TRIM_US = 0
DEFAULT_CH4_TRIM_US = 0

# ======= ★ 직진/회전 문제 해결 핵심 파라미터 =======
# 직진 시 CH4=1500(정중앙)이 브레이크/무시로 해석되는 문제를 피하기 위한 미세 바이어스/게인
DEFAULT_STRAIGHT_CH4_BIAS_US = 20   # 전진/후진 때 CH4에 줄 ±오프셋(10~30 권장)
DEFAULT_STRAIGHT_CH4_GAIN_US = 0    # t에 비례한 오프셋(µs). 0이면 비활성

# 회전-only 때 CH2에 얹는 베이스 스로틀(동력)
DEFAULT_SPIN_BOOST_NORM = 0.35      # 0~1 권장 (예: 0.35)
DEFAULT_USE_SPIN_SIGN   = False     # 회전 부호에 따라 앞/뒤로 줄지(보통 False=항상 전진쪽)

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

        self.declare_parameter("debug_log",       DEFAULT_LOG_LEVEL_DEBUG)
        self.declare_parameter("arming_style",    DEFAULT_ARMING_STYLE)
        self.declare_parameter("force_thr_us",    DEFAULT_FORCE_THR_US)

        self.declare_parameter("mode",                DEFAULT_MODE)
        self.declare_parameter("tank_k",              DEFAULT_TANK_K)
        self.declare_parameter("invert_turn_dir",     DEFAULT_INVERT_TURN_DIR)

        self.declare_parameter("invert_ch2",    DEFAULT_INVERT_CH2)
        self.declare_parameter("invert_ch4",    DEFAULT_INVERT_CH4)
        self.declare_parameter("swap_channels", DEFAULT_SWAP_CHANNELS)

        self.declare_parameter("ch2_scale",   DEFAULT_CH2_SCALE)
        self.declare_parameter("ch4_scale",   DEFAULT_CH4_SCALE)
        self.declare_parameter("ch2_trim_us", DEFAULT_CH2_TRIM_US)
        self.declare_parameter("ch4_trim_us", DEFAULT_CH4_TRIM_US)

        # ★ 핵심 파라미터
        self.declare_parameter("straight_ch4_bias_us", DEFAULT_STRAIGHT_CH4_BIAS_US)
        self.declare_parameter("straight_ch4_gain_us", DEFAULT_STRAIGHT_CH4_GAIN_US)
        self.declare_parameter("spin_boost_norm",      DEFAULT_SPIN_BOOST_NORM)
        self.declare_parameter("use_spin_sign",        DEFAULT_USE_SPIN_SIGN)

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

        self.debug_log    = bool(self.get_parameter("debug_log").value)
        self.arming_style = str(self.get_parameter("arming_style").value).strip().lower()
        self.force_thr_us = int(self.get_parameter("force_thr_us").value)

        self.mode          = str(self.get_parameter("mode").value).strip().lower()
        self.tank_k        = float(self.get_parameter("tank_k").value)
        self.invert_turn   = bool(self.get_parameter("invert_turn_dir").value)

        self.invert_ch2    = bool(self.get_parameter("invert_ch2").value)
        self.invert_ch4    = bool(self.get_parameter("invert_ch4").value)
        self.swap_channels = bool(self.get_parameter("swap_channels").value)

        self.ch2_scale   = float(self.get_parameter("ch2_scale").value)
        self.ch4_scale   = float(self.get_parameter("ch4_scale").value)
        self.ch2_trim_us = int(self.get_parameter("ch2_trim_us").value)
        self.ch4_trim_us = int(self.get_parameter("ch4_trim_us").value)

        self.straight_ch4_bias_us = int(self.get_parameter("straight_ch4_bias_us").value)
        self.straight_ch4_gain_us = int(self.get_parameter("straight_ch4_gain_us").value)
        self.spin_boost_norm      = float(self.get_parameter("spin_boost_norm").value)
        self.use_spin_sign        = bool(self.get_parameter("use_spin_sign").value)

        if self.arming_style not in ("neutral", "min"):
            self.get_logger().warn(f"Unknown arming_style '{self.arming_style}', fallback to 'neutral'")
            self.arming_style = "neutral"

        # 내부 상태
        self.last_thr_us = self.neutral
        self.rev_state   = RevGateState.IDLE
        self.rev_gate_until = None

        # GPIO 초기화
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.thr_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.steer_pin, GPIO.OUT, initial=GPIO.LOW)

        # PWM
        self.pwm_thr   = GPIO.PWM(self.thr_pin, self.f_hz)
        self.pwm_steer = GPIO.PWM(self.steer_pin, self.f_hz)

        # 아밍 펄스
        self.arming_us = self.neutral if self.arming_style == "neutral" else min(self.fwd_far, self.neutral)
        self.pwm_thr.start(self._us_to_duty(self.arming_us))
        self.pwm_steer.start(self._us_to_duty(self.neutral))

        now = self.get_clock().now()
        self.arming_until = now + Duration(seconds=self.arm_s)
        self.last_rcv     = now

        # ROS I/O
        self.sub   = self.create_subscription(Twist, self.topic, self.on_cmd, 10)
        self.timer = self.create_timer(self.wd_ms / 1000.0, self.watchdog)

        self.get_logger().info(
            f"[throttle_steer_node] CH2=BOARD{self.thr_pin}, CH4=BOARD{self.steer_pin}, "
            f"freq={self.f_hz}Hz, neutral={self.neutral}us, mode={self.mode}, "
            f"straight_bias={self.straight_ch4_bias_us}us, spin_boost={self.spin_boost_norm}"
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
        """정규화 [-1..1] -> ESC 펄스폭 (CH2용)"""
        v = max(-1.0, min(1.0, v))
        if abs(v) < max(1e-3, self.deadband / 1000.0):
            return self.neutral
        if v > 0.0:
            us = int(self.fwd_near + (self.fwd_far - self.fwd_near) * v)     # 1500→1000
        else:
            us = int(self.rev_near + (self.rev_far - self.rev_near) * (-v))  # 1500→2000
        return self._clamp_us(us)

    def _map_steer(self, s: float) -> int:
        """정규화 [-1..1] -> 스티어 펄스폭 (CH4용)"""
        s = max(-1.0, min(1.0, s))
        return self._clamp_us(self.neutral + int(s * self.steer_max))

    def _apply_pwm(self, ch2_us: int, ch4_us: int):
        self.pwm_thr.ChangeDutyCycle(self._us_to_duty(ch2_us))
        self.pwm_steer.ChangeDutyCycle(self._us_to_duty(ch4_us))

    def _maybe_invert(self, us: int, invert: bool) -> int:
        # 1500us 기준 대칭 반전
        return 2 * self.neutral - us if invert else us

    def _apply_cal(self, us: int, scale: float, trim_us: int) -> int:
        # 1500 기준 스케일+트림
        return int(self.neutral + scale * (us - self.neutral) + trim_us)

    # ===== 콜백 =====
    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()

        # 아밍 동안
        if now < self.arming_until:
            self._apply_pwm(self.arming_us, self.neutral)
            return

        # 입력 정규화
        t = max(-1.0, min(1.0, msg.linear.x))   # 속도(전/후진) → CH2
        s = max(-1.0, min(1.0, msg.angular.z))  # 방향(회전)     → CH4
        if self.invert_turn:
            s = -s

        # ---- simple_split: CH2=속도, CH4=방향 ----
        t_dead = max(1e-3, self.deadband / 1000.0)
        rotate_only = (abs(t) < t_dead) and (abs(s) >= t_dead)

        if rotate_only:
            # 회전-only: CH4가 방향, CH2는 약간의 동력(스핀 부스트)
            ch4_us = self._map_steer(s)
            spin_mag = max(0.0, min(1.0, self.spin_boost_norm)) * abs(s)
            if self.use_spin_sign:
                spin_t = spin_mag if s >= 0.0 else -spin_mag
            else:
                spin_t = spin_mag  # 항상 전진쪽
            ch2_us = self._map_esc(spin_t)

        else:
            # 전/후진: CH2만 속도, CH4는 '완전 1500' 회피용 작은 오프셋
            ch2_us = self._map_esc(t)
            ch4_off = 0
            if self.straight_ch4_bias_us != 0:
                ch4_off += self.straight_ch4_bias_us if t > 0.0 else (-self.straight_ch4_bias_us if t < 0.0 else 0)
            if self.straight_ch4_gain_us != 0:
                ch4_off += int(self.straight_ch4_gain_us * t)
            ch4_us = self._clamp_us(self.neutral + ch4_off)

        # ===== 보정/반전/스왑 적용 순서 =====
        # (1) 스케일/트림
        ch2_us = self._apply_cal(ch2_us, self.ch2_scale, self.ch2_trim_us)
        ch4_us = self._apply_cal(ch4_us, self.ch4_scale, self.ch4_trim_us)
        # (2) 채널 반전
        ch2_us = self._maybe_invert(ch2_us, self.invert_ch2)
        ch4_us = self._maybe_invert(ch4_us, self.invert_ch4)
        # (3) 스왑
        if self.swap_channels:
            ch2_us, ch4_us = ch4_us, ch2_us

        # 적용
        if self.debug_log:
            self.get_logger().info(f"[cmd] t={t:.2f} s={s:.2f} -> ch2={ch2_us} ch4={ch4_us}")
        self._apply_pwm(ch2_us, ch4_us)

        self.last_rcv = now
        self.last_thr_us = ch2_us

    def watchdog(self):
        if (self.get_clock().now() - self.last_rcv).nanoseconds > self.wd_ms * 1_000_000:
            nd = self._us_to_duty(self.neutral)
            self.pwm_thr.ChangeDutyCycle(nd)
            self.pwm_steer.ChangeDutyCycle(nd)
            self.rev_state = RevGateState.IDLE

    def destroy_node(self):
        try:
            self.pwm_thr.stop()
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
