import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))
print("listening on 127.0.0.1:5005")
try:
    while True:
        data, addr = sock.recvfrom(4096)
        try:
            print("recv from", addr, json.loads(data.decode("utf-8")))
        except Exception:
            print("recv raw:", data)
except KeyboardInterrupt:
    print("stopped")
    sock.close()
