import socket
import json
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from .kalman_imu import KalmanIMU

# UDP setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except Exception:
    pass
sock.bind(("127.0.0.1", 5005))
sock.setblocking(False)

# state
pos = np.zeros(3)        # meters
vel = np.zeros(3)        # m/s
# quaternion: [w, x, y, z] (identity = no rotation)
q = np.array([1.0, 0.0, 0.0, 0.0])
last_t = None

g_mps2 = 9.80665  # convert g->m/s^2 if accel in g

# helpers
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_normalize(qv):
    return qv / np.linalg.norm(qv)

def integrate_gyro(qv, omega, dt):
    # omega = angular rate vector in rad/s (wx,wy,wz)
    # quaternion derivative: q_dot = 0.5 * q * [0, wx, wy, wz]
    omega_q = np.array([0.0, omega[0], omega[1], omega[2]])
    q_dot = 0.5 * quat_mul(qv, omega_q)
    q_new = qv + q_dot * dt
    return quat_normalize(q_new)

def quat_rotate(qv, v):
    # rotate vector v (3,) by quaternion qv
    q_conj = np.array([qv[0], -qv[1], -qv[2], -qv[3]])
    v_q = np.array([0.0, v[0], v[1], v[2]])
    return quat_mul(quat_mul(qv, v_q), q_conj)[1:]

# nonblocking receive -> returns latest packet dict or None
def recv_latest():
    last = None
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            pkt = json.loads(data.decode("utf-8"))
            last = pkt
        except BlockingIOError:
            break
        except Exception:
            break
    return last

# plotting setup
fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
traj_x, traj_y, traj_z = [], [], []
line, = ax.plot([], [], [], '-o', lw=1)

# status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")


def recv_latest():
    last = None
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            pkt = json.loads(data.decode("utf-8"))
            last = pkt
            print("recv pkt:", pkt)          # terminal debug
        except BlockingIOError:
            break
        except Exception:
            break
    return last

# Start of code for arrow visual!
orient_line = None
orient_head_lines = []

def update(frame):
    global last_t, q, pos, vel, orient_line, orient_head_lines
    now = time.time()
    if last_t is None:
        last_t = now
    dt = now - last_t
    if dt <= 0:
        dt = 1e-3
    last_t = now

    pkt = recv_latest()
    if pkt is not None:
        # ...existing code...
        traj_x.append(pos[0])
        traj_y.append(pos[1])
        traj_z.append(pos[2])

    # update plot limits adaptively
    all_x = traj_x[-200:]
    all_y = traj_y[-200:]
    all_z = traj_z[-200:]
    if all_x:
        ax.set_xlim(min(all_x)-1, max(all_x)+1)
        ax.set_ylim(min(all_y)-1, max(all_y)+1)
        ax.set_zlim(min(all_z)-1, max(all_z)+1)

    line.set_data(traj_x, traj_y)
    line.set_3d_properties(traj_z)

    # draw orientation arrow (body forward = +X)
    body_forward = np.array([0.5, 0.0, 0.0])  # arrow nominal length direction
    world_forward = quat_rotate(q, body_forward)

    # remove previous arrow artists
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

    # compute arrow shaft and triangular head
    L = np.linalg.norm(world_forward)
    if L < 1e-6:
        # nothing to draw yet
        return (line,)

    forward_norm = world_forward / L
    tip = pos + forward_norm * 1.0   # scale arrow tip distance (1.0 m)
    tail = pos

    # shaft
    orient_line, = ax.plot([tail[0], tip[0]],
                           [tail[1], tip[1]],
                           [tail[2], tip[2]],
                           color='r', lw=2)

    # head parameters
    head_len = 0.2  # meters back from tip
    head_width = 0.12

    # pick a vector orthogonal to forward_norm
    up = np.array([0.0, 0.0, 1.0])
    orth = np.cross(forward_norm, up)
    if np.linalg.norm(orth) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
        orth = np.cross(forward_norm, up)
    orth = orth / np.linalg.norm(orth) * head_width

    # second orthogonal direction
    orth2 = np.cross(forward_norm, orth)
    orth2 = orth2 / np.linalg.norm(orth2) * (head_width * 0.6)

    base = tip - forward_norm * head_len
    left = base + orth + orth2
    right = base - orth + orth2
    down = base - orth2

    # draw triangular head (3 lines from tip to three base points)
    h1, = ax.plot([tip[0], left[0]], [tip[1], left[1]], [tip[2], left[2]], color='r', lw=2)
    h2, = ax.plot([tip[0], right[0]], [tip[1], right[1]], [tip[2], right[2]], color='r', lw=2)
    h3, = ax.plot([tip[0], down[0]], [tip[1], down[1]], [tip[2], down[2]], color='r', lw=2)
    orient_head_lines = [h1, h2, h3]

    return (line, orient_line, *orient_head_lines)

ani = animation.FuncAnimation(fig, update, interval=100)
plt.show()