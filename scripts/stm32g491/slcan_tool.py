#!/usr/bin/env python3
"""Minimal SLCAN client for the FDCAN chardev of the STM32G491 machine.

Connects to the TCP chardev (-chardev socket,id=fdcan2,host=...,port=...,
server=on,wait=off), sends the given frames and prints every frame it sees
for --wait seconds. Frames are SLCAN text (the "t<iii><l><data>" /
"T<iiiiiiii><l><data>" format); use --ext-id/--data for convenience.

    slcan_tool.py --connect 127.0.0.1:6210 --ext-id 0x1F --data 0000000000000000
    slcan_tool.py --connect 127.0.0.1:6210 --send T0000001F800000000000000000

Python 3 standard library only.
"""
import argparse
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--connect", required=True, metavar="HOST:PORT")
    ap.add_argument("--send", action="append", default=[], metavar="FRAME",
                    help="raw SLCAN frame text (without the trailing CR)")
    ap.add_argument("--ext-id", type=lambda s: int(s, 0), help="send an extended frame with this ID")
    ap.add_argument("--data", default="", help="hex data bytes for --ext-id")
    ap.add_argument("--wait", type=float, default=1.0, help="seconds to listen")
    ap.add_argument("--delay", type=float, default=0.0, help="seconds to wait before sending")
    args = ap.parse_args()

    host, port = args.connect.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=5)
    s.settimeout(0.1)
    frames = list(args.send)
    if args.ext_id is not None:
        n = len(args.data) // 2
        frames.append("T%08X%d%s" % (args.ext_id, n, args.data.upper()))
    t0 = time.time()
    sent = False
    buf = b""
    while time.time() - t0 < args.wait + args.delay:
        if not sent and time.time() - t0 >= args.delay:
            for f in frames:
                s.sendall(f.encode() + b"\r")
            sent = True
        try:
            d = s.recv(4096)
        except socket.timeout:
            continue
        if not d:
            break
        buf += d
        while b"\r" in buf:
            line, buf = buf.split(b"\r", 1)
            if line:
                print("%.3f %s" % (time.time() - t0, line.decode(errors="replace")))
                sys.stdout.flush()
    return 0


def run():
    try:
        return main()
    except BrokenPipeError:
        return 0


if __name__ == "__main__":
    sys.exit(run())
