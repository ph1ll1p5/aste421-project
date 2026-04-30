"""
kalman_imu.py — 6-DOF Kalman Filter for IMU Sensor Fusion
Fuses (ax, ay, az) + (gx, gy, gz) → (roll, pitch, yaw)

No magnetometer. Yaw is gyro-integrated and will drift slowly over time.
Roll and pitch are corrected by the accelerometer and remain stable.

Theory:
  State vector:  x = [roll, pitch, yaw, bias_gx, bias_gy, bias_gz]
  
  Gyro measures angular rate corrupted by a slowly-drifting bias.
  Accelerometer provides a gravity-referenced correction for roll/pitch only.
  Yaw has no absolute reference — it integrates from gyro with bias removed.

Usage:
    kf = KalmanIMU(dt=0.02)           # 50 Hz sample rate
    roll, pitch, yaw = kf.update(ax, ay, az, gx, gy, gz)

Units:
    ax/ay/az : m/s²
    gx/gy/gz : rad/s   ← convert deg/s with math.radians() if needed
    Output   : degrees
"""

import math
import numpy as np


class KalmanIMU:
    """
    Extended Kalman Filter for attitude estimation.

    State: [roll, pitch, yaw, b_gx, b_gy, b_gz]  (all radians internally)

    Process model:
        angle_new = angle_old + (gyro - bias) * dt
        bias_new  = bias_old                        (random walk)

    Measurement model (accelerometer):
        Provides roll and pitch reference from gravity vector.
        Yaw is unobservable without a magnetometer.
    """

    def __init__(self, dt=0.02,
                 gyro_noise=0.003,       # rad/s  — gyro measurement noise
                 gyro_bias_noise=0.0001, # rad/s² — how fast bias drifts
                 accel_noise=0.5):       # m/s²   — accelerometer noise
        """
        Args:
            dt              : sample period in seconds (1 / sample_rate_hz)
            gyro_noise      : std dev of gyro measurement noise (rad/s)
                              Increase if gyro is noisy; decrease for smoother.
            gyro_bias_noise : std dev of gyro bias random walk (rad/s²)
                              Increase if bias drifts fast (warm-up drift).
            accel_noise     : std dev of accelerometer noise (m/s²)
                              Increase in high-vibration environments.
        """
        self.dt = dt

        # ── State vector: [roll, pitch, yaw, b_gx, b_gy, b_gz] ──
        self.x = np.zeros(6)

        # ── State covariance ──
        self.P = np.eye(6) * 0.1

        # ── Process noise covariance Q ──
        q_angle = gyro_noise ** 2
        q_bias  = gyro_bias_noise ** 2
        self.Q = np.diag([q_angle, q_angle, q_angle,
                          q_bias,  q_bias,  q_bias])

        # ── Measurement noise covariance R (roll, pitch only) ──
        r = accel_noise ** 2
        self.R = np.diag([r, r])

        # ── Measurement matrix H: maps state → [roll, pitch] ──
        self.H = np.zeros((2, 6))
        self.H[0, 0] = 1.0  # roll
        self.H[1, 1] = 1.0  # pitch

        self._initialized = False

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, ax, ay, az, gx, gy, gz):
        """
        Ingest one IMU sample and return fused attitude.

        Args:
            ax, ay, az : accelerometer (m/s²)
            gx, gy, gz : gyroscope     (rad/s)

        Returns:
            (roll_deg, pitch_deg, yaw_deg)
        """
        if not self._initialized:
            self._initialize_from_accel(ax, ay, az)
            self._initialized = True
            return self._angles_deg()

        # 1. Predict
        self._predict(gx, gy, gz)

        # 2. Update from accelerometer (if signal is trustworthy)
        g_mag = math.sqrt(ax**2 + ay**2 + az**2)
        # Only correct from accel when magnitude is close to 1g
        # (ignore during high-acceleration maneuvers)
        if 7.0 < g_mag < 13.0:
            roll_accel, pitch_accel = self._accel_to_angles(ax, ay, az)
            self._correct(roll_accel, pitch_accel)

        return self._angles_deg()

    def reset(self):
        self.x = np.zeros(6)
        self.P = np.eye(6) * 0.1
        self._initialized = False

    def set_dt(self, dt):
        """Update sample period (call if your data rate changes)."""
        self.dt = dt

    @property
    def bias(self):
        """Current estimated gyro bias (rad/s)."""
        return self.x[3:6].copy()

    # ── Internal steps ────────────────────────────────────────────────────────

    def _initialize_from_accel(self, ax, ay, az):
        """Bootstrap roll/pitch from first accelerometer reading."""
        roll, pitch = self._accel_to_angles(ax, ay, az)
        self.x[0] = roll
        self.x[1] = pitch
        self.x[2] = 0.0   # yaw unknown, start at 0

    def _predict(self, gx, gy, gz):
        dt = self.dt
        roll, pitch, yaw, b_gx, b_gy, b_gz = self.x

        # Remove estimated bias
        wx = gx - b_gx
        wy = gy - b_gy
        wz = gz - b_gz

        # Euler angle kinematics (body rates → Euler rates)
        # For small angles this simplifies, but we keep the full form:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        tp = math.tan(pitch)

        roll_dot  = wx + sr * tp * wy + cr * tp * wz
        pitch_dot =           cr * wy -      sr * wz
        yaw_dot   = (sr / cp) * wy + (cr / cp) * wz if abs(cp) > 1e-6 else 0.0

        # State transition
        self.x[0] += roll_dot  * dt
        self.x[1] += pitch_dot * dt
        self.x[2] += yaw_dot   * dt
        # biases unchanged (random walk — noise handles it)

        # Wrap angles to [-π, π]
        self.x[0] = self._wrap(self.x[0])
        self.x[1] = self._wrap(self.x[1])
        self.x[2] = self._wrap(self.x[2])

        # Jacobian of state transition F
        F = np.eye(6)
        # ∂roll_dot/∂roll, ∂roll_dot/∂pitch
        F[0, 0] = 1 + dt * (cr * tp * wy - sr * tp * wz)
        F[0, 1] = dt * (sr / (cp**2) * wy + cr / (cp**2) * wz) if abs(cp) > 1e-6 else 0
        # ∂roll_dot/∂biases
        F[0, 3] = -dt
        F[0, 4] = -dt * sr * tp
        F[0, 5] = -dt * cr * tp
        # ∂pitch_dot/∂roll
        F[1, 0] = dt * (-sr * wy - cr * wz)
        F[1, 4] = -dt * cr
        F[1, 5] =  dt * sr
        # ∂yaw_dot/∂roll, ∂yaw_dot/∂pitch
        if abs(cp) > 1e-6:
            F[2, 0] = dt * (cr / cp * wy - sr / cp * wz)
            F[2, 1] = dt * (sr * sp / cp**2 * wy + cr * sp / cp**2 * wz)
            F[2, 4] = -dt * sr / cp
            F[2, 5] = -dt * cr / cp

        # Propagate covariance
        self.P = F @ self.P @ F.T + self.Q

    def _correct(self, roll_meas, pitch_meas):
        """Kalman update step using accelerometer-derived roll/pitch."""
        z = np.array([roll_meas, pitch_meas])
        y = z - self.H @ self.x                  # innovation
        y[0] = self._wrap(y[0])
        y[1] = self._wrap(y[1])

        S = self.H @ self.P @ self.H.T + self.R  # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S) # Kalman gain

        self.x = self.x + K @ y
        self.x[0] = self._wrap(self.x[0])
        self.x[1] = self._wrap(self.x[1])
        self.x[2] = self._wrap(self.x[2])

        self.P = (np.eye(6) - K @ self.H) @ self.P

    @staticmethod
    def _accel_to_angles(ax, ay, az):
        """Compute roll and pitch from accelerometer (radians)."""
        roll  = math.atan2(ay, math.sqrt(ax**2 + az**2))
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        return roll, pitch

    @staticmethod
    def _wrap(angle):
        """Wrap angle to [-π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _angles_deg(self):
        return (math.degrees(self.x[0]),
                math.degrees(self.x[1]),
                math.degrees(self.x[2]))


# ─── Convenience wrapper: handles deg/s input ─────────────────────────────────

class KalmanIMUDegrees(KalmanIMU):
    """Same as KalmanIMU but accepts gyro in deg/s instead of rad/s."""

    def update(self, ax, ay, az, gx_deg, gy_deg, gz_deg):
        return super().update(ax, ay, az,
                              math.radians(gx_deg),
                              math.radians(gy_deg),
                              math.radians(gz_deg))


# ─── Standalone test / tuning tool ────────────────────────────────────────────

if __name__ == "__main__":
    import argparse, csv, sys, time

    parser = argparse.ArgumentParser(description="Test Kalman IMU filter")
    parser.add_argument("--csv",  help="CSV file with columns: ax,ay,az,gx,gy,gz[,dt]")
    parser.add_argument("--demo", action="store_true", help="Synthetic demo")
    parser.add_argument("--gyro-deg", action="store_true",
                        help="Gyro input is deg/s (default: rad/s)")
    parser.add_argument("--dt",   type=float, default=0.02, help="Sample period (s)")
    parser.add_argument("--gyro-noise",  type=float, default=0.003)
    parser.add_argument("--accel-noise", type=float, default=0.5)
    args = parser.parse_args()

    FilterClass = KalmanIMUDegrees if args.gyro_deg else KalmanIMU
    kf = FilterClass(dt=args.dt,
                     gyro_noise=args.gyro_noise,
                     accel_noise=args.accel_noise)

    print(f"{'TIME':>8} {'ROLL':>8} {'PITCH':>8} {'YAW':>8}  "
          f"{'B_GX':>8} {'B_GY':>8} {'B_GZ':>8}")
    print("-" * 72)

    if args.csv:
        with open(args.csv) as f:
            reader = csv.DictReader(f)
            t = 0.0
            for row in reader:
                ax = float(row["ax"]); ay = float(row["ay"]); az = float(row["az"])
                gx = float(row["gx"]); gy = float(row["gy"]); gz = float(row["gz"])
                dt = float(row.get("dt", args.dt))
                kf.set_dt(dt)
                r, p, y = kf.update(ax, ay, az, gx, gy, gz)
                bx, by, bz = kf.bias
                print(f"{t:8.3f} {r:+8.2f} {p:+8.2f} {y:+8.2f}  "
                      f"{math.degrees(bx):+8.4f} {math.degrees(by):+8.4f} {math.degrees(bz):+8.4f}")
                t += dt

    elif args.demo:
        t = 0.0
        while True:
            # Simulate tilting rocket
            ax =  math.sin(t * 0.3) * 2.0
            ay =  math.sin(t * 0.2) * 1.5
            az = -9.81 + math.sin(t * 0.5) * 0.3
            gx =  math.cos(t * 0.3) * math.radians(20)
            gy =  math.cos(t * 0.2) * math.radians(15)
            gz =  math.radians(5)   # slow yaw spin

            r, p, y = kf.update(ax, ay, az, gx, gy, gz)
            bx, by, bz = kf.bias
            print(f"{t:8.3f} {r:+8.2f} {p:+8.2f} {y:+8.2f}  "
                  f"{math.degrees(bx):+8.4f} {math.degrees(by):+8.4f} {math.degrees(bz):+8.4f}",
                  end="\r")
            t += args.dt
            time.sleep(args.dt)
    else:
        print("Use --demo or --csv <file>. Run with -h for help.")
