import socket
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))
sock.setblocking(False)

latest = {
    "accel_x": -1.0,
    "accel_y": 0.0,
    "accel_z": 0.0,
    "ang_x": 0.5,
    "ang_y": 0.6,
    "ang_z": 0.7,
}

def recv_packets():
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            try:
                pkt = json.loads(data.decode("utf-8"))
                latest.update(pkt)
            except Exception:
                pass
        except BlockingIOError:
            break
        except Exception:
            break

fig, ax = plt.subplots(figsize=(6,4))
bars = ax.bar(["accel_x","accel_y","accel_z"],
              [latest["accel_x"], latest["accel_y"], latest["accel_z"]],
              color=["C0","C1","C2"])
ax.set_ylim(-2.0, 2.0)
ax.set_ylabel("g (approx)")
angle_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top")

def update(frame):
    recv_packets()
    acc = [latest["accel_x"], latest["accel_y"], latest["accel_z"]]
    for b, h in zip(bars, acc):
        b.set_height(h)
    angle_text.set_text(f"ang_x={latest['ang_x']:.2f}  ang_y={latest['ang_y']:.2f}  ang_z={latest['ang_z']:.2f}")
    return (*bars, angle_text)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.title("Live IMU visual (accel bars + angles)")
plt.tight_layout()
plt.show()