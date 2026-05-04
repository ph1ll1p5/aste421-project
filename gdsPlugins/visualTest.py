import socket
import json
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from gdsPlugins.kalman_imu import KalmanIMUDegrees

# ----------------------------
# Settings
# ----------------------------
HOST = "127.0.0.1"
PORT = 5005

# Set this to True if accel_x/y/z come in g instead of m/s^2
ACCEL_IN_G = True

# Set this to True if ang_x/y/z come in deg/s
GYRO_IN_DEG_PER_SEC = True

GRAVITY = 9.80665

# Kalman filter for attitude
kf = KalmanIMUDegrees(dt=0.1) if GYRO_IN_DEG_PER_SEC else None

# ----------------------------
# Quaternion helpers
# ----------------------------
def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    # [w, x, y, z]
    return np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    ], dtype=float)

def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)

def quat_rotate(qv, v):
    q_conj = np.array([qv[0], -qv[1], -qv[2], -qv[3]], dtype=float)
    v_q = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(qv, v_q), q_conj)[1:]

# ----------------------------
# UDP setup
# ----------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except Exception:
    pass

sock.bind((HOST, PORT))
sock.setblocking(False)

def recv_latest():
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            pkt = json.loads(data.decode("utf-8"))
            latest = pkt
        except BlockingIOError:
            break
        except Exception:
            break
    return latest

# ----------------------------
# State
# ----------------------------
q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # orientation quaternion
pos = np.zeros(3, dtype=float)                   # optional position estimate
vel = np.zeros(3, dtype=float)                   # optional velocity estimate
last_t = None

# ----------------------------
# Plot setup
# ----------------------------
fig = plt.figure(figsize=(8, 7))
ax3d = fig.add_subplot(111, projection="3d")
ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")

traj_x, traj_y, traj_z = [], [], []
path_line, = ax3d.plot([], [], [], "-o", lw=1)

orient_line = None
orient_head_lines = []

status_text = ax3d.text2D(0.02, 0.98, "", transform=ax3d.transAxes, va="top")

def update(frame):
    global q, pos, vel, last_t, orient_line, orient_head_lines

    now = time.time()
    if last_t is None:
        last_t = now
    dt = now - last_t
    if dt <= 0:
        dt = 1e-3
    last_t = now

    pkt = recv_latest()
    if pkt is not None:
        # Read packet fields
        acc_x = float(pkt.get("accel_x", 0.0))
        acc_y = float(pkt.get("accel_y", 0.0))
        acc_z = float(pkt.get("accel_z", 0.0))
        gyro_x = float(pkt.get("ang_x", 0.0))
        gyro_y = float(pkt.get("ang_y", 0.0))
        gyro_z = float(pkt.get("ang_z", 0.0))

        # Convert units if needed
        if ACCEL_IN_G:
            acc_x *= GRAVITY
            acc_y *= GRAVITY
            acc_z *= GRAVITY

        # Update attitude estimate
        if GYRO_IN_DEG_PER_SEC:
            roll, pitch, yaw = kf.update(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
        else:
            # If gyro is rad/s, use KalmanIMU directly
            from gdsPlugins.kalman_imu import KalmanIMU
            if not hasattr(update, "_kf_rad"):
                update._kf_rad = KalmanIMU(dt=0.1)
            roll, pitch, yaw = update._kf_rad.update(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

        q = euler_to_quat(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw),
        )

        # Optional simple motion estimate:
        # Rotate body accel to world frame and integrate.
        # This will drift over time, but it makes the trail move.
        world_acc = quat_rotate(q, np.array([acc_x, acc_y, acc_z], dtype=float))
        lin_acc = world_acc - np.array([0.0, 0.0, GRAVITY], dtype=float)

        vel += lin_acc * dt
        vel *= 0.995  # light damping to reduce runaway drift
        pos += vel * dt

        traj_x.append(pos[0])
        traj_y.append(pos[1])
        traj_z.append(pos[2])

        status_text.set_text(
            f"roll={roll:7.2f}  pitch={pitch:7.2f}  yaw={yaw:7.2f}\n"
            f"ax={acc_x:7.2f} ay={acc_y:7.2f} az={acc_z:7.2f}"
        )

    # Update path limits
    if traj_x:
        recent_x = traj_x[-200:]
        recent_y = traj_y[-200:]
        recent_z = traj_z[-200:]
        ax3d.set_xlim(min(recent_x) - 1, max(recent_x) + 1)
        ax3d.set_ylim(min(recent_y) - 1, max(recent_y) + 1)
        ax3d.set_zlim(min(recent_z) - 1, max(recent_z) + 1)

    path_line.set_data(traj_x, traj_y)
    path_line.set_3d_properties(traj_z)

    # Draw orientation arrow (body forward = +X)
    body_forward = np.array([0.5, 0.0, 0.0], dtype=float)
    world_forward = quat_rotate(q, body_forward)

    if orient_line is not None:
        try:
            orient_line.remove()
        except Exception:
            pass

    for h in orient_head_lines:
        try:
            h.remove()
        except Exception:
            pass
    orient_head_lines = []

    norm = np.linalg.norm(world_forward)
    if norm < 1e-6:
        return (path_line,)

    forward_norm = world_forward / norm
    tail = pos.copy()
    tip = tail + forward_norm * 1.0

    orient_line, = ax3d.plot(
        [tail[0], tip[0]],
        [tail[1], tip[1]],
        [tail[2], tip[2]],
        color="r",
        lw=2,
    )

    # Arrow head
    head_len = 0.2
    head_width = 0.12

    up = np.array([0.0, 0.0, 1.0])
    orth = np.cross(forward_norm, up)
    if np.linalg.norm(orth) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
        orth = np.cross(forward_norm, up)

    orth = orth / np.linalg.norm(orth) * head_width
    orth2 = np.cross(forward_norm, orth)
    orth2 = orth2 / np.linalg.norm(orth2) * (head_width * 0.6)

    base = tip - forward_norm * head_len
    left = base + orth + orth2
    right = base - orth + orth2
    down = base - orth2

    h1, = ax3d.plot([tip[0], left[0]], [tip[1], left[1]], [tip[2], left[2]], color="r", lw=2)
    h2, = ax3d.plot([tip[0], right[0]], [tip[1], right[1]], [tip[2], right[2]], color="r", lw=2)
    h3, = ax3d.plot([tip[0], down[0]], [tip[1], down[1]], [tip[2], down[2]], color="r", lw=2)
    orient_head_lines = [h1, h2, h3]

    return (path_line, orient_line, *orient_head_lines)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.tight_layout()
plt.show()