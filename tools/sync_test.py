#!/usr/bin/env python3
"""Art-Net frame-sync tester.

Alternates black/white DMX frames to two target IPs with a configurable stagger,
and optionally broadcasts an OpSync packet after both targets have been updated.

Use to validate the WLED OpSync receive path: with --no-artsync each node
should latch its frame on its own (so the slow target visibly trails the fast
one); with --artsync both should latch together.

Examples:
  # Stress test without sync — both nodes should flicker out of phase
  ./sync_test.py 192.168.1.10 192.168.1.11

  # Same with OpSync emission — should appear in lockstep
  ./sync_test.py 192.168.1.10 192.168.1.11 --artsync

  # 30 LEDs each, universes 0 and 1, 50ms stagger, 250ms color flip
  ./sync_test.py 10.0.0.5 10.0.0.6 --leds 30 --universes 0 1 \\
      --stagger-ms 50 --interval-ms 250 --artsync
"""

import argparse
import socket
import struct
import sys
import time

ARTNET_PORT = 6454
ARTNET_HEADER = b"Art-Net\x00"
PROTOVER = (0x00, 0x0e)


def build_dmx(universe: int, sequence: int, data: bytes) -> bytes:
    # opcode OpDmx 0x5000 transmitted little-endian on the wire (0x00, 0x50)
    if len(data) % 2:
        data += b"\x00"  # length must be even
    return (
        ARTNET_HEADER
        + bytes((0x00, 0x50))
        + bytes(PROTOVER)
        + bytes((sequence & 0xFF, 0x00))            # sequence, physical
        + struct.pack("<H", universe & 0xFFFF)       # universe (LE)
        + struct.pack(">H", len(data))               # data length (BE)
        + data
    )


def build_sync() -> bytes:
    # OpSync 0x5200 LE on the wire (0x00, 0x52)
    return (
        ARTNET_HEADER
        + bytes((0x00, 0x52))
        + bytes(PROTOVER)
        + bytes((0x00, 0x00))                        # aux1, aux2 (transmit as 0)
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("targets", nargs=2, metavar="IP",
                   help="two WLED target IPs")
    p.add_argument("--universes", type=int, nargs="+", default=[0, 1],
                   help="universe(s) to send to each target. One value = same on both, "
                        "two values = per-target. Default: 0 1")
    p.add_argument("--leds", type=int, default=120,
                   help="LED count per universe (RGB, 3 channels each). Default: 120")
    p.add_argument("--full-universe", action="store_true",
                   help="send a full 510-byte payload (170 RGB LEDs); overrides --leds")
    p.add_argument("--interval-ms", type=int, default=500,
                   help="ms between black/white flips. Default: 500")
    p.add_argument("--stagger-ms", type=int, default=100,
                   help="ms delay between sending to first and second target. Default: 100")
    p.add_argument("--artsync", action="store_true",
                   help="broadcast OpSync after both DMX packets are sent")
    p.add_argument("--artsync-delay-us", type=int, default=0,
                   help="microsecond pause before broadcasting OpSync. Default: 0")
    p.add_argument("--broadcast", default="255.255.255.255",
                   help="OpSync broadcast destination. Default: 255.255.255.255")
    args = p.parse_args()

    if len(args.universes) == 1:
        unis = [args.universes[0], args.universes[0]]
    elif len(args.universes) == 2:
        unis = args.universes
    else:
        print("--universes takes 1 or 2 values", file=sys.stderr)
        return 2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    if args.full_universe:
        channels = 510  # 170 RGB LEDs, even byte count, max single-universe payload
        led_count = 170
    else:
        channels = args.leds * 3
        led_count = args.leds
    black = bytes(channels)
    white = b"\xff" * channels

    print(f"Targets : {args.targets[0]} (uni {unis[0]}), {args.targets[1]} (uni {unis[1]})")
    print(f"LEDs    : {led_count} per universe ({channels} channels)")
    print(f"Flip    : every {args.interval_ms} ms")
    print(f"Stagger : {args.stagger_ms} ms between targets")
    if args.artsync:
        print(f"OpSync  : broadcast to {args.broadcast}"
              + (f" after {args.artsync_delay_us} us" if args.artsync_delay_us else ""))
    else:
        print("OpSync  : disabled")
    print("Ctrl+C to stop.\n")

    seq = 1
    on = False
    try:
        while True:
            on = not on
            data = white if on else black
            tag = "WHITE" if on else "BLACK"
            t0 = time.perf_counter()

            sock.sendto(build_dmx(unis[0], seq, data), (args.targets[0], ARTNET_PORT))
            t_first = time.perf_counter()

            if args.stagger_ms > 0:
                time.sleep(args.stagger_ms / 1000.0)

            sock.sendto(build_dmx(unis[1], seq, data), (args.targets[1], ARTNET_PORT))
            t_second = time.perf_counter()

            if args.artsync:
                if args.artsync_delay_us > 0:
                    time.sleep(args.artsync_delay_us / 1_000_000.0)
                sock.sendto(build_sync(), (args.broadcast, ARTNET_PORT))
                t_sync = time.perf_counter()
                print(f"[{tag}] seq={seq:>3} "
                      f"a={1000*(t_first-t0):5.1f}ms "
                      f"b={1000*(t_second-t0):5.1f}ms "
                      f"sync={1000*(t_sync-t0):5.1f}ms")
            else:
                print(f"[{tag}] seq={seq:>3} "
                      f"a={1000*(t_first-t0):5.1f}ms "
                      f"b={1000*(t_second-t0):5.1f}ms")

            seq = (seq % 255) + 1  # 1..255 wrap, never 0 (0 = sequence disabled)

            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            remaining = args.interval_ms - elapsed_ms
            if remaining > 0:
                time.sleep(remaining / 1000.0)
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
