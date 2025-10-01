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
DEFAULT_FWD_NEAR_US  = 1500   # t=0 → 1500
DEFAULT_FWD_FAR_US   = 1000   # t=+1 → 1000
DEFAULT_REV_NEAR_US  = 1500   # t=0 → 1500
DEFAULT_REV_FAR_US   = 2000   # t=-1 → 2000

# 로깅/아밍/강제
DEFAULT_LOG_LEVEL_DEBUG = True
DEFAULT_ARMING_STYLE    = "neutral"   # "neutral"|"min"
DEFAULT_FORCE_THR_US    = 0           # 0=off

# 동작 모드(확장 대비 — 현재는 simple_split 사용)
DEFAULT_MODE            = "simple_split"   # "simple_split" | "nav_tank_mix"
DEFAULT_TANK_K          = 0.5
DEFAULT_INVERT_TURN_DIR = False

# 채널 반전/스왑
DEFAULT_INVERT_CH2    = False
DEFAULT_INVERT_CH4    = False
DEFAULT_SWAP_CHANNELS = False

# 밸런스 보정
DEFAULT_CH2_SCALE   = 1.0
DEFAULT_CH4_SCALE   = 1.0
DEFAULT_CH2_TRIM_US = 0
DEFAULT_CH4_TRIM_US = 0

# 회전-only 때 CH2 동력 보조(스핀 부스트)
DEFAULT_SPIN_BOOST_NORM = 0.35   # 0~1
DEFAULT_USE_SPIN_SIGN   = False  # False면 항상 전진쪽 부스트

# CH4 사용 여부
DEFAULT_ENABLE_STEER = True


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

        self.declare_parameter("spin_boost_norm", DEFAULT_SPIN_BOOST_NORM)
        self.declare_parameter("use_spin_sign",   DEFAULT_USE_SPIN_SIGN)

        self.declare_parameter("enable_steer", DEFAULT_ENABLE_STEER)

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

        self.spin_boost_norm = float(self.get_parameter("spin_boost_norm").value)
        self.use_spin_sign   = bool(self.get_parameter("use_spin_sign").value)

        self.enable_steer = bool(self.get_parameter("enable_steer").value)

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

        # CH2(스로틀) — 항상 사용
        GPIO.setup(self.thr_pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_thr = GPIO.PWM(self.thr_pin, self.f_hz)

        # CH4(스티어) — 방어로직 포함
        self.pwm_steer = None
        self.steer_pwm_active = False
        self.steer_configured = False

        if self.enable_steer:
            try:
                GPIO.setup(self.steer_pin, GPIO.OUT, initial=GPIO.LOW)
                # setup 성공 시 점 먼저 True로
                self.steer_configured = True
                # 무신호 유지
                GPIO.output(self.steer_pin, GPIO.LOW)
                # PWM 객체 생성
                self.pwm_steer = GPIO.PWM(self.steer_pin, self.f_hz)
            except Exception as e:
                self.get_logger().warn(
                    f"CH4(BOARD{self.steer_pin}) setup 실패 → enable_steer=False로 전환: {e}"
                )
                self.enable_steer = False
                self.pwm_steer = None
                self.steer_configured = False

        # 아밍: CH2 시작, CH4는 설정된 경우에만 무신호로
        self.arming_us = self.neutral if self.arming_style == "neutral" else min(self.fwd_far, self.neutral)
        self.pwm_thr.start(self._us_to_duty(self.arming_us))

        if self.enable_steer and self.steer_configured:
            try:
                GPIO.output(self.steer_pin, GPIO.LOW)  # 무신호
            except Exception as e:
                self.get_logger().warn(f"CH4 무신호 설정 실패 → CH4 비활성화로 전환: {e}")
                self.enable_steer = False
                self.steer_configured = False
                self.pwm_steer = None

        now = self.get_clock().now()
        self.arming_until = now + Duration(seconds=self.arm_s)
        self.last_rcv     = now

        # ROS I/O
        self.sub   = self.create_subscription(Twist, self.topic, self.on_cmd, 10)
        self.timer = self.create_timer(self.wd_ms / 1000.0, self.watchdog)

        self.get_logger().info(
            f"[throttle_steer_node] CH2=BOARD{self.thr_pin}, "
            f"CH4={'BOARD'+str(self.steer_pin) if (self.enable_steer and self.steer_configured) else 'DISABLED'}, "
            f"freq={self.f_hz}Hz, neutral={self.neutral}us, mode={self.mode}, "
            f"spin_boost={self.spin_boost_norm}, enable_steer={self.enable_steer}"
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
        """정규화 [-1..1] -> ESC 펄스폭 (CH2)"""
        v = max(-1.0, min(1.0, v))
        if abs(v) < max(1e-3, self.deadband / 1000.0):
            return self.neutral
        if v > 0.0:
            us = int(self.fwd_near + (self.fwd_far - self.fwd_near) * v)     # 1500→1000
        else:
            us = int(self.rev_near + (self.rev_far - self.rev_near) * (-v))  # 1500→2000
        return self._clamp_us(us)

    def _map_steer(self, s: float) -> int:
        """정규화 [-1..1] -> 스티어 펄스폭 (CH4)"""
        s = max(-1.0, min(1.0, s))
        return self._clamp_us(self.neutral + int(s * self.steer_max))

    def _maybe_invert(self, us: int, invert: bool) -> int:
        return 2 * self.neutral - us if invert else us

    def _apply_cal(self, us: int, scale: float, trim_us: int) -> int:
        return int(self.neutral + scale * (us - self.neutral) + trim_us)

    def _steer_pwm_start(self, us: int):
        if not (self.enable_steer and self.steer_configured and self.pwm_steer):
            return
        duty = self._us_to_duty(us)
        if not self.steer_pwm_active:
            try:
                self.pwm_steer.start(duty)
                self.steer_pwm_active = True
            except Exception as e:
                self.get_logger().warn(f"CH4 PWM start 실패(무시): {e}")
        else:
            self.pwm_steer.ChangeDutyCycle(duty)

    def _steer_pwm_stop(self):
        if not (self.enable_steer and self.steer_configured and self.pwm_steer):
            return
        if self.steer_pwm_active:
            try:
                self.pwm_steer.stop()
            except Exception as e:
                self.get_logger().warn(f"CH4 PWM stop 실패(무시): {e}")
            self.steer_pwm_active = False
        try:
            GPIO.output(self.steer_pin, GPIO.LOW)  # 무신호
        except Exception:
            pass

    def _apply_pwm(self, ch2_us: int, ch4_us: int, apply_ch4: bool):
        """apply_ch4=False면 CH4는 건드리지 않음(OFF 유지)."""
        self.pwm_thr.ChangeDutyCycle(self._us_to_duty(ch2_us))
        if apply_ch4 and self.enable_steer and self.steer_configured and self.pwm_steer and self.steer_pwm_active:
            self.pwm_steer.ChangeDutyCycle(self._us_to_duty(ch4_us))

    # ===== 콜백 =====
    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()

        # 아밍 동안: CH2만 유지, CH4는 OFF
        if now < self.arming_until:
            self._steer_pwm_stop()
            self._apply_pwm(self.arming_us, self.neutral, apply_ch4=False)
            return

        # 입력 정규화
        t = max(-1.0, min(1.0, msg.linear.x))
        s = max(-1.0, min(1.0, msg.angular.z))
        if self.invert_turn:
            s = -s

        # 직진/회전 분기
        t_dead = max(1e-3, self.deadband / 1000.0)
        rotate_only = (abs(t) < t_dead) and (abs(s) >= t_dead)

        if rotate_only:
            # 회전-only: CH4 ON, CH2는 스핀부스트
            ch4_raw = self._map_steer(s)
            self._steer_pwm_start(ch4_raw)

            spin_mag = max(0.0, min(1.0, self.spin_boost_norm)) * abs(s)
            if self.use_spin_sign:
                spin_t = spin_mag if s >= 0.0 else -spin_mag
            else:
                spin_t = spin_mag
            ch2_raw = self._map_esc(spin_t)

            # 보정/반전/스왑
            ch2_us = self._apply_cal(ch2_raw, self.ch2_scale, self.ch2_trim_us)
            ch4_us = self._apply_cal(ch4_raw, self.ch4_scale, self.ch4_trim_us)
            ch2_us = self._maybe_invert(ch2_us, self.invert_ch2)
            ch4_us = self._maybe_invert(ch4_us, self.invert_ch4)
            if self.swap_channels:
                ch2_us, ch4_us = ch4_us, ch2_us

            if self.debug_log:
                self.get_logger().info(f"[cmd] ROT t={t:.2f} s={s:.2f} -> ch2={ch2_us} ch4={ch4_us} (CH4 ON)")
            self._apply_pwm(ch2_us, ch4_us, apply_ch4=True)

        else:
            # 직진/후진: CH4 OFF, CH2만 사용
            self._steer_pwm_stop()
            ch2_raw = self._map_esc(t)

            ch2_us = self._apply_cal(ch2_raw, self.ch2_scale, self.ch2_trim_us)
            ch2_us = self._maybe_invert(ch2_us, self.invert_ch2)
            ch4_us = self.neutral  # 로깅용

            if self.debug_log:
                self.get_logger().info(f"[cmd] STR t={t:.2f} s={s:.2f} -> ch2={ch2_us} ch4=OFF")
            self._apply_pwm(ch2_us, ch4_us, apply_ch4=False)

        self.last_rcv = now
        self.last_thr_us = ch2_us

    def watchdog(self):
        # 입력 끊기면 CH2 중립, CH4 OFF
        if (self.get_clock().now() - self.last_rcv).nanoseconds > self.wd_ms * 1_000_000:
            self._steer_pwm_stop()
            nd = self._us_to_duty(self.neutral)
            self.pwm_thr.ChangeDutyCycle(nd)
            self.rev_state = RevGateState.IDLE

    def destroy_node(self):
        try:
            self.pwm_thr.stop()
            self._steer_pwm_stop()
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
