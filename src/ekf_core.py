#!/usr/bin/env python3
"""
ekf_core.py
===========
9-state Extended Kalman Filter for UAV INS.
Direct Python port of ekf_core.m from ARYA-mgc/ins-system-for-drone.

State vector  x = [px, py, pz,  vx, vy, vz,  phi, theta, psi]  (9×1)
                   └── pos(NED)─┘  └── vel(NED)─┘  └──Euler(rad)──┘

Inputs
------
  accel  : specific force in body frame  [ax, ay, az]  m/s²
  gyro   : angular rate in body frame    [p,  q,  r ]  rad/s
  dt     : time step                     seconds

Aiding sensors (update steps)
------------------------------
  Barometer  → measures z (altitude, NED negative-up convention)
  Magnetometer → measures psi (yaw)
"""

import numpy as np
from imu_noise_params import IMUNoiseParams

# Gravity vector (NED, m/s²)
GRAVITY = np.array([0.0, 0.0, 9.80665])


# ════════════════════════════════════════════════════════════════
class EKFCore:
    """
    Extended Kalman Filter — 9-DOF strapdown INS.
    """

    def __init__(self, noise: IMUNoiseParams):
        self.noise = noise

        # ── State ──────────────────────────────────────────────
        # x = [px py pz vx vy vz phi theta psi]
        self.x = np.zeros(9)

        # ── Covariance ─────────────────────────────────────────
        self.P = np.diag([
            1.0, 1.0, 1.0,          # pos uncertainty  (m²)
            0.1, 0.1, 0.1,          # vel uncertainty  (m/s)²
            0.01, 0.01, 0.01,       # att uncertainty  (rad²)
        ])

        # ── Process noise Q ────────────────────────────────────
        sa = noise.accel_std ** 2
        sg = noise.gyro_std  ** 2
        # build 9×9 Q (diagonal approximation)
        self.Q = np.diag([
            0.5*sa, 0.5*sa, 0.5*sa,   # position (driven by vel noise)
            sa,     sa,     sa,        # velocity
            sg,     sg,     sg,        # attitude
        ]) * 1e-2   # scale tuning

        # ── Baro measurement noise ─────────────────────────────
        self.R_baro = np.array([[noise.baro_std ** 2]])

        # ── Mag measurement noise ──────────────────────────────
        self.R_mag  = np.array([[noise.mag_std ** 2]])

        # ── Observation matrices (linear) ─────────────────────
        # Baro observes z (index 2)
        self.H_baro = np.zeros((1, 9))
        self.H_baro[0, 2] = 1.0

        # Mag observes psi (index 8)
        self.H_mag = np.zeros((1, 9))
        self.H_mag[0, 8] = 1.0

    # ── convenience properties ──────────────────────────────────
    @property
    def state(self) -> dict:
        return {
            "pos":   self.x[0:3].copy(),
            "vel":   self.x[3:6].copy(),
            "euler": self.x[6:9].copy(),
        }

    # ── rotation matrix body→NED ───────────────────────────────
    @staticmethod
    def _R_bn(phi: float, theta: float, psi: float) -> np.ndarray:
        """
        Direction-cosine matrix (body → NED).
        Tait-Bryan 3-2-1 (psi=yaw, theta=pitch, phi=roll).
        """
        cp, sp = np.cos(phi),   np.sin(phi)
        ct, st = np.cos(theta), np.sin(theta)
        cy, sy = np.cos(psi),   np.sin(psi)

        R = np.array([
            [ct*cy,  sp*st*cy - cp*sy,  cp*st*cy + sp*sy],
            [ct*sy,  sp*st*sy + cp*cy,  cp*st*sy - sp*cy],
            [-st,    sp*ct,             cp*ct           ],
        ])
        return R

    # ── Euler kinematics matrix T⁻¹ ───────────────────────────
    @staticmethod
    def _T_inv(phi: float, theta: float) -> np.ndarray:
        """
        Maps body angular rates → Euler rate.
        η̇ = T⁻¹ · ω_body
        Singularity at theta = ±90° (gimbal lock — acceptable for UAV).
        """
        cp, sp = np.cos(phi),   np.sin(phi)
        ct, st = np.cos(theta), np.sin(theta)
        # protect against gimbal lock
        if abs(ct) < 1e-6:
            ct = 1e-6

        return np.array([
            [1.0,  sp*st/ct,  cp*st/ct],
            [0.0,  cp,        -sp      ],
            [0.0,  sp/ct,     cp/ct   ],
        ])

    # ═══════════════════════════════════════════════════════════
    # PREDICT — strapdown mechanisation + covariance propagation
    # ═══════════════════════════════════════════════════════════
    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """
        EKF predict step (called at every IMU sample).

        Equations (NED frame):
          v̇  = R_bn · f_body + g
          ṗ  = v
          η̇  = T⁻¹ · ω_body
        """
        px, py, pz = self.x[0:3]
        vx, vy, vz = self.x[3:6]
        phi, theta, psi = self.x[6:9]

        R = self._R_bn(phi, theta, psi)
        T = self._T_inv(phi, theta)

        # ── state propagation ──────────────────────────────────
        # velocity update
        f_ned = R @ accel                        # specific force in NED
        acc_ned = f_ned + GRAVITY                # add gravity (NED: +z down)
        vel_new = self.x[3:6] + acc_ned * dt

        # position update
        pos_new = self.x[0:3] + self.x[3:6] * dt

        # attitude update
        eta_dot = T @ gyro
        euler_new = self.x[6:9] + eta_dot * dt

        # wrap angles to [-π, π]
        euler_new[0] = self._wrap_angle(euler_new[0])
        euler_new[1] = self._wrap_angle(euler_new[1])
        euler_new[2] = self._wrap_angle(euler_new[2])

        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        self.x[6:9] = euler_new

        # ── Jacobian F (linearised state transition) ───────────
        F = self._compute_F(accel, phi, theta, psi, dt)

        # ── covariance propagation P = F P Fᵀ + Q ─────────────
        self.P = F @ self.P @ F.T + self.Q

    # ── Jacobian of state transition ──────────────────────────
    def _compute_F(self, accel, phi, theta, psi, dt) -> np.ndarray:
        """
        Linearised state-transition Jacobian (9×9).
        dẋ/dx evaluated at current state.
        """
        F = np.eye(9)

        # dp/dv
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        R = self._R_bn(phi, theta, psi)
        f_ned = R @ accel

        # dv/deuler  (partial of R_bn*accel w.r.t. euler angles)
        # Numerical Jacobian (small enough for 9-state system)
        eps = 1e-5
        for i in range(3):          # phi, theta, psi
            euler_p = np.array([phi, theta, psi])
            euler_p[i] += eps
            Rp = self._R_bn(*euler_p)
            df = (Rp @ accel - f_ned) / eps
            F[3, 6+i] = df[0] * dt
            F[4, 6+i] = df[1] * dt
            F[5, 6+i] = df[2] * dt

        # dη/dη  (T⁻¹ contribution — identity + small correction)
        # For small dt, the attitude block ≈ I; full Jacobian omitted
        # for simplicity (common in practice for MEMS-grade systems).

        return F

    # ═══════════════════════════════════════════════════════════
    # UPDATE — barometer (altitude)
    # ═══════════════════════════════════════════════════════════
    def update_baro(self, alt_measured: float):
        """
        EKF measurement update using barometer altitude.
        H = [0 0 1  0 0 0  0 0 0]  (observes z = pz)

        Note: NED z is positive downward.
        Barometer gives MSL altitude (positive up).
        We negate: pz = -alt_measured.
        """
        z_meas = np.array([-alt_measured])   # NED z
        z_pred = self.H_baro @ self.x

        y   = z_meas - z_pred                # innovation
        S   = self.H_baro @ self.P @ self.H_baro.T + self.R_baro
        K   = self.P @ self.H_baro.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(9) - K @ self.H_baro) @ self.P

        self.x[6:9] = [self._wrap_angle(a) for a in self.x[6:9]]

    # ═══════════════════════════════════════════════════════════
    # UPDATE — magnetometer (yaw)
    # ═══════════════════════════════════════════════════════════
    def update_mag(self, yaw_measured: float):
        """
        EKF measurement update using magnetometer yaw.
        H = [0 0 0  0 0 0  0 0 1]  (observes psi)
        Innovation is wrapped to [-π, π] to avoid wrap-around jumps.
        """
        z_meas = np.array([yaw_measured])
        z_pred = np.array([self.x[8]])

        y = np.array([self._wrap_angle(z_meas[0] - z_pred[0])])

        S = self.H_mag @ self.P @ self.H_mag.T + self.R_mag
        K = self.P @ self.H_mag.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(9) - K @ self.H_mag) @ self.P

        self.x[6:9] = [self._wrap_angle(a) for a in self.x[6:9]]

    # ── util ───────────────────────────────────────────────────
    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap angle to [-π, π]."""
        import math
        return math.atan2(math.sin(a), math.cos(a))

    def reset(self):
        """Reset state and covariance to initial values."""
        self.__init__(self.noise)
