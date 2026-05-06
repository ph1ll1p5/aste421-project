#!/usr/bin/env python3
# ...existing code...
import argparse
import asyncio
import json
from aiohttp import web, WSCloseCode
import pathlib
import socket

WS_SET = set()

class UDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            pkt = json.loads(data.decode("utf-8"))
        except Exception:
            return
        payload = json.dumps(pkt)
        for ws in list(WS_SET):
            asyncio.create_task(send_ws(ws, payload))

async def send_ws(ws, payload):
    try:
        await ws.send_str(payload)
    except Exception:
        try:
            await ws.close(code=WSCloseCode.GOING_AWAY)
        except Exception:
            pass
        WS_SET.discard(ws)

async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    WS_SET.add(ws)
    try:
        async for _ in ws:
            pass
    finally:
        WS_SET.discard(ws)
    return ws

async def index_handler(request):
    return web.FileResponse(path=request.app["static_index"])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--http-port", type=int, default=9000)
    args = parser.parse_args()

    loop = asyncio.get_event_loop()

    # UDP listener
    listen = loop.create_datagram_endpoint(UDPProtocol, local_addr=(args.host, args.port))
    transport, _ = loop.run_until_complete(listen)
    print(f"UDP listening on {args.host}:{args.port}")

    # aiohttp app
    app = web.Application()
    base = pathlib.Path(__file__).parent / "static"
    app["static_index"] = str(base / "index.html")
    app.router.add_get("/", index_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static/", path=str(base), name="static")

    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site_http = web.TCPSite(runner, "0.0.0.0", args.http_port)
    site_ws = web.TCPSite(runner, "0.0.0.0", args.ws_port)
    loop.run_until_complete(site_http.start())
    loop.run_until_complete(site_ws.start())
    print(f"HTTP UI at http://127.0.0.1:{args.http_port}/")
    print(f"WebSocket endpoint ws://127.0.0.1:{args.ws_port}/ws")

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        loop.run_until_complete(runner.cleanup())

if __name__ == "__main__":
    main()
# ...existing code...