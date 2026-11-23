"""Listen for read-only FreeRTOS health reports on UDP port 5002."""

import argparse
import json
import socket
import struct
import time


PORT = 5002
REPORT = struct.Struct(">8s9I")
FIELDS = (
    "sequence", "uptime_ms", "max_control_late_us", "max_control_run_us",
    "missed_periods", "vehicle_stack_free_words", "health_stack_free_words",
    "can_online_mask", "ethernet_retries",
)
CAN_SOURCES = ("RT49", "EPS", "SEB", "SEB faults", "EPS fault page 0", "EPS fault page 1")


def decode_report(payload):
    magic, *values = REPORT.unpack(payload)
    if magic != b"hlt__\x01\x00\x00":
        raise ValueError("unknown health report format")
    report = dict(zip(FIELDS, values))
    mask = report["can_online_mask"]
    report["can_without_recent_feedback"] = [
        name for bit, name in enumerate(CAN_SOURCES) if not (mask & (1 << bit))
    ]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.5.11")
    parser.add_argument("--stm32", default="192.168.5.50")
    parser.add_argument("--duration", type=float, default=10)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    count = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
        listener.bind((args.bind, PORT))
        end = time.monotonic() + args.duration
        while (remaining := end - time.monotonic()) > 0:
            listener.settimeout(remaining)
            try:
                payload, sender = listener.recvfrom(2048)
            except socket.timeout:
                break
            if sender != (args.stm32, PORT):
                continue
            try:
                report = decode_report(payload)
            except (ValueError, struct.error):
                continue
            print(json.dumps(report), flush=True)
            count += 1
    print(f"Received {count} health reports")
    return 0 if count else 1


if __name__ == "__main__":
    raise SystemExit(main())
