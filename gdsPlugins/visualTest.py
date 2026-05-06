import socket
import json
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ----------------------------
# Settings
# ----------------------------
HOST = "127.0.0.1"
PORT = 5005

# ----------------------------
# Quaternion helpers
# ----------------------------
def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
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
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=float)

def quat_rotate(q, v):
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_q = np.array([0.0, v[0], v[1], v[2]])
    return quat_mul(quat_mul(q, v_q), q_conj)[1:]

# ----------------------------
# Rocket geometry (body frame)
# +Z = nose, body along Z axis
# ----------------------------
def make_rocket_faces():
    faces = []

    # Body cylinder (8-sided prism)
    r = 0.15
    z_bot = -0.6
    z_top = 0.4
    n = 8
    angles = [2 * math.pi * i / n for i in range(n)]
    body_pts_bot = [(r * math.cos(a), r * math.sin(a), z_bot) for a in angles]
    body_pts_top = [(r * math.cos(a), r * math.sin(a), z_top) for a in angles]

    for i in range(n):
        j = (i + 1) % n
        faces.append((
            np.array([body_pts_bot[i], body_pts_bot[j],
                      body_pts_top[j], body_pts_top[i]]),
            "#888888"
        ))

    # Nose cone
    nose_tip = (0, 0, 0.9)
    for i in range(n):
        j = (i + 1) % n
        faces.append((
            np.array([body_pts_top[i], body_pts_top[j], nose_tip]),
            "#cc4444"
        ))

    # 4 fins
    fin_angles = [0, math.pi / 2, math.pi, 3 * math.pi / 2]
    for a in fin_angles:
        ca, sa = math.cos(a), math.sin(a)
        root_inner = (r * ca,         r * sa,         z_bot)
        root_outer = (r * ca,         r * sa,         z_bot + 0.25)
        tip        = ((r + 0.3) * ca, (r + 0.3) * sa, z_bot)
        faces.append((
            np.array([root_inner, root_outer, tip]),
            "#4444cc"
        ))

    return faces

ROCKET_FACES = make_rocket_faces()

def rotate_faces(faces, q):
    return [(np.array([quat_rotate(q, v) for v in verts]), color)
            for verts, color in faces]

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
            latest = json.loads(data.decode("utf-8"))
        except BlockingIOError:
            break
        except Exception:
            break
    return latest

# ----------------------------
# Plot setup
# ----------------------------
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_zlim(-1.2, 1.2)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("BNO055 Rocket Orientation (Fused)")

# Static world-frame reference axes
ax.quiver(0, 0, 0, 1, 0, 0, color='red',   alpha=0.2, linewidth=1)
ax.quiver(0, 0, 0, 0, 1, 0, color='green', alpha=0.2, linewidth=1)
ax.quiver(0, 0, 0, 0, 0, 1, color='blue',  alpha=0.2, linewidth=1)

status_text = ax.text2D(0.02, 0.98, "Waiting for data...",
                         transform=ax.transAxes, va="top",
                         fontsize=10, family="monospace")

poly_collection = None
q = np.array([1.0, 0.0, 0.0, 0.0])  # identity — rocket starts upright


def update(frame):
    global q, poly_collection

    pkt = recv_latest()
    if pkt is not None:
        # BNO055 fused euler angles packed into accel fields by ImuHelpers.cpp:
        #   acceleration.x = roll, acceleration.y = pitch, acceleration.z = yaw
        roll = float(pkt.get("roll",  0.0))
        pitch = float(pkt.get("pitch", 0.0))
        yaw = float(pkt.get("yaw",   0.0))

        q = euler_to_quat(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw),
        )

        status_text.set_text(
            f"roll  = {roll:+7.2f}°\n"
            f"pitch = {pitch:+7.2f}°\n"
            f"yaw   = {yaw:+7.2f}°\n"
            f"seq   = {pkt.get('seq', 0)}"
        )

    # Remove old rocket geometry
    if poly_collection is not None:
        poly_collection.remove()

    # Draw rotated rocket
    rotated = rotate_faces(ROCKET_FACES, q)
    verts  = [r[0] for r in rotated]
    colors = [r[1] for r in rotated]

    poly_collection = Poly3DCollection(verts, alpha=0.85)
    poly_collection.set_facecolor(colors)
    poly_collection.set_edgecolor("#333333")
    ax.add_collection3d(poly_collection)

    return (poly_collection,)


ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
plt.show()