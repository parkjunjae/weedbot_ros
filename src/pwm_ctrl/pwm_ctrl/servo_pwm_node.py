#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from math import cos, sin
from rclpy.clock import Clock
from transforms3d.euler import euler2quat  # 패키지: python3-tf-transformations
import tf2_ros

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
DEFAULT_STEER_MAX    = 300      # 스티어 최대 변위(µs) = ±300 → 1200~1800
DEFAULT_DBAND_US     = 30

# ==== ESC 맵 (정규화 입력 [-1,1]) ====
# 대칭 맵: 중립 1500, 전진/후진 끝값 1000/2000
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
        
        # -------odom에서 base_link계산 파라미터 --------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_pub_topic", "/atoz/odom")  # 슬램이 lookup하는 frame만 맞으면 토픽명은 자유
        self.declare_parameter("odom_rate_hz", 50.0)       # 적분/퍼블리시 주기
        self.declare_parameter("max_linear_mps", 1.0)      # t=+1.0일 때 선속도 [m/s]
        self.declare_parameter("max_angular_rps", 1.0)     # s=+1.0일 때 각속도 [rad/s]

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
        
        # -------odom에서 base_link계산 로드 --------
        self.odom_frame   = str(self.get_parameter("odom_frame").value)
        self.base_frame   = str(self.get_parameter("base_frame").value)
        self.odom_topic   = str(self.get_parameter("odom_pub_topic").value)
        self.odom_rate    = float(self.get_parameter("odom_rate_hz").value)
        self.v_max        = float(self.get_parameter("max_linear_mps").value)
        self.w_max        = float(self.get_parameter("max_angular_rps").value)
        
        
        # Odometry publisher & TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 적분 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # 최근 명령(정규화 → 물리 속도)
        self.cmd_v = 0.0   # [m/s]
        self.cmd_w = 0.0   # [rad/s]

        # 오도메트리 타이머
        self.odom_timer = self.create_timer(1.0 / self.odom_rate, self._tick_odom)
        

        if self.arming_style not in ("neutral", "min"):
            self.get_logger().warn(f"Unknown arming_style '{self.arming_style}', fallback to 'neutral'")
            self.arming_style = "neutral"

        # 내부 상태
        self.last_thr_us = self.neutral
        self.rev_state   = RevGateState.IDLE
        self.rev_gate_until = None

        # ===== GPIO 초기화 =====
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # CH2(스로틀) — 항상 사용
        GPIO.setup(self.thr_pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_thr = GPIO.PWM(self.thr_pin, self.f_hz)

        # CH4(스티어) — ★지연 초기화★
        self.pwm_steer     = None
        self.steer_ready   = False   # setup + PWM.start 가 끝난 상태?
        self.steer_enabled = self.enable_steer  # 런타임 플래그

        # 아밍: CH2만 시작 (CH4는 손대지 않음)
        self.arming_us = self.neutral if self.arming_style == "neutral" else min(self.fwd_far, self.neutral)
        self.pwm_thr.start(self._us_to_duty(self.arming_us))

        now = self.get_clock().now()
        self.arming_until = now + Duration(seconds=self.arm_s)
        self.last_rcv     = now

        # ROS I/O
        self.sub   = self.create_subscription(Twist, self.topic, self.on_cmd, 10)
        self.timer = self.create_timer(self.wd_ms / 1000.0, self.watchdog)

        self.get_logger().info(
            f"[throttle_steer_node] CH2=BOARD{self.thr_pin}, "
            f"CH4={'BOARD'+str(self.steer_pin) if self.steer_enabled else 'LAZY/OFF'}, "
            f"freq={self.f_hz}Hz, neutral={self.neutral}us, mode={self.mode}, "
            f"spin_boost={self.spin_boost_norm}, enable_steer={self.steer_enabled}"
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

    # ===== CH4 지연 초기화/해제 =====
    def _steer_pwm_start(self, us: int):
        """회전 시작/갱신 시 호출. 준비 안되었으면 여기서 setup → PWM.start."""
        if not self.steer_enabled:
            return
        duty = self._us_to_duty(us)
        if not self.steer_ready:
            try:
                GPIO.setup(self.steer_pin, GPIO.OUT, initial=GPIO.LOW)
                self.pwm_steer = GPIO.PWM(self.steer_pin, self.f_hz)
                self.pwm_steer.start(duty)
                self.steer_ready = True
                if self.debug_log:
                    self.get_logger().info(f"[CH4] lazy-setup & PWM.start @ {us}us")
            except Exception as e:
                # CH4 비활성화로 전환
                self.get_logger().warn(f"CH4 lazy-setup 실패 → CH4 비활성화: {e}")
                self.steer_enabled = False
                self.pwm_steer = None
                self.steer_ready = False
                return
        else:
            # 이미 PWM 가동 중이면 듀티만 갱신
            self.pwm_steer.ChangeDutyCycle(duty)

    def _steer_pwm_stop(self):
        """회전 종료 시 호출. PWM만 멈추고, 필요 시 채널 cleanup."""
        if not self.steer_enabled:
            return
        if self.steer_ready and self.pwm_steer is not None:
            try:
                self.pwm_steer.stop()
            except Exception:
                pass
            self.pwm_steer = None
            self.steer_ready = False
            # 선택: 채널 반납(핀 완전 해제) — 회전 시작 때 다시 setup
            try:
                GPIO.cleanup(self.steer_pin)
            except Exception:
                pass
            if self.debug_log:
                self.get_logger().info("[CH4] PWM.stop & cleanup(pin)")

    # ===== PWM 적용 =====
    def _apply_pwm(self, ch2_us: int):
        """CH2만 항상 적용. CH4는 회전일 때만 _steer_pwm_start/stop 에서 관리."""
        self.pwm_thr.ChangeDutyCycle(self._us_to_duty(ch2_us))

    # ===== 콜백 =====
    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()

        # 아밍 동안: CH2만 유지, CH4는 손대지 않음
        if now < self.arming_until:
            self._steer_pwm_stop()   # 혹시라도 켜져있다면 정리
            self._apply_pwm(self.arming_us)
            self.cmd_v = 0.0
            self.cmd_w = 0.0
            return

        # 입력 정규화
        t = max(-1.0, min(1.0, msg.linear.x))   # 속도(전/후진) → CH2
        s = max(-1.0, min(1.0, msg.angular.z))  # 방향(회전)     → CH4
        if self.invert_turn:
            s = -s

        # 직진/회전 분기
        t_dead = max(1e-3, self.deadband / 1000.0)
        rotate_only = (abs(t) < t_dead) and (abs(s) >= t_dead)

        if rotate_only and self.steer_enabled:
            # 회전-only: CH4 ON, CH2는 스핀부스트
            ch4_raw = self._map_steer(s)
            self._steer_pwm_start(ch4_raw)

            spin_mag = max(0.0, min(1.0, self.spin_boost_norm)) * abs(s)
            spin_t = (spin_mag if self.use_spin_sign and s >= 0.0
                      else (-spin_mag if self.use_spin_sign else spin_mag))
            ch2_raw = self._map_esc(spin_t)

            # 보정/반전/스왑(실질적으로 CH2만 의미 있음)
            ch2_us = self._apply_cal(ch2_raw, self.ch2_scale, self.ch2_trim_us)
            ch2_us = self._maybe_invert(ch2_us, self.invert_ch2)

            if self.swap_channels:
                # 스왑 의미가 크지 않지만, 유지
                pass

            if self.debug_log:
                self.get_logger().info(f"[cmd] ROT t={t:.2f} s={s:.2f} -> ch2={ch2_us} ch4~{ch4_raw}")
            self._apply_pwm(ch2_us)

        else:
            # 직진/후진: CH4 OFF(아예 손 떼기), CH2만 사용
            self._steer_pwm_stop()
            ch2_raw = self._map_esc(t)
            ch2_us = self._apply_cal(ch2_raw, self.ch2_scale, self.ch2_trim_us)
            ch2_us = self._maybe_invert(ch2_us, self.invert_ch2)

            if self.debug_log:
                self.get_logger().info(f"[cmd] STR t={t:.2f} s={s:.2f} -> ch2={ch2_us} ch4=OFF")
            self._apply_pwm(ch2_us)

        self.last_rcv = now
        self.last_thr_us = ch2_us
        
        # ---- 속도 명령 보관 ------
        self.cmd_v = t * self.v_max
        self.cmd_w = s * self.w_max

    def watchdog(self):
        # 입력 끊기면 CH2 중립, CH4 OFF(정리)
        if (self.get_clock().now() - self.last_rcv).nanoseconds > self.wd_ms * 1_000_000:
            self._steer_pwm_stop()
            nd = self._us_to_duty(self.neutral)
            self.pwm_thr.ChangeDutyCycle(nd)
            self.rev_state = RevGateState.IDLE
            self.cmd_v = 0.0
            self.cmd_w = 0.0

    def destroy_node(self):
        try:
            self.pwm_thr.stop()
            self._steer_pwm_stop()
        finally:
            GPIO.cleanup()
            super().destroy_node()
            
    def _tick_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            # 큰 점프는 버퍼/슬립 등으로 발생할 수 있음 → 상태만 갱신
            self.last_time = now
            return
        # 적분 (평면)
        self.x   += self.cmd_v * cos(self.yaw) * dt
        self.y   += self.cmd_v * sin(self.yaw) * dt
        self.yaw += self.cmd_w * dt
    
        # 쿼터니언
        w, x, y, z = euler2quat(0.0, 0.0, self.yaw, axes='sxyz')
        qx, qy, qz, qw = x, y, z, w
    
        # Odometry 메시지
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id  = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        # 속도 (base_link 기준)
        od.twist.twist.linear.x  = self.cmd_v
        od.twist.twist.linear.y  = 0.0
        od.twist.twist.angular.z = self.cmd_w
        # 대략적 공분산 (필요시 파라미터화)
        cov = [0.0]*36
        cov[0]  = 1e-3  # x
        cov[7]  = 1e-3  # y
        cov[35] = 1e-2  # yaw
        cov[14] = 1e6   # z
        cov[21] = 1e6   # roll
        cov[28] = 1e6   # pitch
        od.pose.covariance = cov
        od.twist.covariance = cov
        self.odom_pub.publish(od)
    
        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
    
        self.last_time = now


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
