#!/usr/bin/env python3
"""Join SLCAN sockets into one CAN bus.

Connects to each node's socket (for example an FDCAN controller of the
stm32h745 machine exposed with -chardev socket,...,server=on, or a UART-based
SLCAN node in another QEMU instance) and forwards every frame line to all the
other nodes, like a CAN bus. With --log, writes one JSON line per frame.

    slcan_hub.py a=127.0.0.1:5600 b=127.0.0.1:5601 [--log frames.jsonl]

SPDX-License-Identifier: GPL-2.0-or-later
"""
import argparse
import json
import selectors
import socket
import sys
import time


def decode(line):
    if not line or line[0] not in "tTrR":
        return None
    idlen = 8 if line[0] in "TR" else 3
    try:
        cid = int(line[1:1 + idlen], 16)
        dlc = int(line[1 + idlen], 16)
    except ValueError:
        return None
    return {"id": f"{cid:X}", "extended": line[0] in "TR", "rtr": line[0] in "rR",
            "dlc": dlc, "data": line[2 + idlen:2 + idlen + 2 * dlc]}


def connect(name, addr, timeout_s):
    host, port = addr.rsplit(":", 1)
    deadline = time.time() + timeout_s
    while True:
        try:
            s = socket.create_connection((host, int(port)))
            s.setblocking(False)
            print(f"slcan_hub: {name} connected ({addr})", flush=True)
            return s
        except OSError:
            if time.time() > deadline:
                sys.exit(f"slcan_hub: cannot connect to {name} at {addr}")
            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("nodes", nargs="+", help="name=host:port")
    ap.add_argument("--log", help="write frames as JSON lines")
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    args = ap.parse_args()

    sel = selectors.DefaultSelector()
    nodes = {}
    for spec in args.nodes:
        name, addr = spec.split("=", 1)
        s = connect(name, addr, args.connect_timeout)
        nodes[name] = {"sock": s, "buf": b""}
        sel.register(s, selectors.EVENT_READ, name)

    log = open(args.log, "w") if args.log else None
    t0 = time.time()
    while nodes:
        for key, _ in sel.select(timeout=1.0):
            name = key.data
            node = nodes[name]
            try:
                chunk = node["sock"].recv(4096)
            except BlockingIOError:
                continue
            if not chunk:
                print(f"slcan_hub: {name} disconnected", flush=True)
                sel.unregister(node["sock"])
                del nodes[name]
                continue
            node["buf"] += chunk
            while b"\r" in node["buf"]:
                line, node["buf"] = node["buf"].split(b"\r", 1)
                line = line.strip()
                if not line:
                    continue
                for other, peer in nodes.items():
                    if other != name:
                        peer["sock"].sendall(line + b"\r")
                if log:
                    rec = {"t": round(time.time() - t0, 6), "src": name,
                           "line": line.decode(errors="replace")}
                    rec.update(decode(rec["line"]) or {})
                    log.write(json.dumps(rec) + "\n")
                    log.flush()


if __name__ == "__main__":
    main()
