"""Listen to real CAN feedback sta__ requirements V1.0 (56 bytes). No commands by default.

python udp_status_test.py --duration 10
Bench receive-path probe (actuators disconnected): add --probe.
Probe sends disabled cmd26 every 20ms; existing firmware still sends EPS center.
"""
import argparse
import contextlib
import datetime
import json
import select
import socket
import struct
import time

VALID_SPEED = 2
VALID_STEERING = 4
VALID_ENCODER = 1
VALID_STEERING_RAW = 8
VALID_BRAKE = 16
VALID_RT49 = 32
VALID_EPS = 64
VALID_SEB = 128

def build_probe(counter):
    # Preserve existing cmd26 byte meanings: version, disabled mode, flags, mask.
    frame = b"cmd__" + bytes([1, 0, 0, 0]) + struct.pack(">ii", 0, 0)
    frame += bytes([0]) + struct.pack(">h", 0) + bytes([0, counter & 255])
    return frame + struct.pack(">I", sum(frame) & 0xffffffff)

def parse_status(frame):
    if len(frame) != 56 or frame[:6] != b"sta__\x01":
        raise ValueError("Expected requirements V1.0 sta__56; previous48/128 unsupported")
    if struct.unpack_from(">I", frame, 52)[0] != sum(frame[:52]) & 0xffffffff:
        raise ValueError("Checksum mismatch")
    valid = frame[6]
    faults = struct.unpack_from(">I", frame, 44)[0]
    brake_mode = frame[34]
    brake_raw = struct.unpack_from(">h", frame, 35)[0]
    enables = {}
    for name, bit in [("rt49",0),("eps",1),("seb",2)]:
        enables[name] = "INVALID" if faults & (1 << (13+bit)) else bool(frame[8] & (1 << bit))
    return {
        "valid_flags": "0x{:02X}".format(valid),
        "source": frame[7], "enable_actual": enables,
        "counter": frame[9],
        "timestamp_ms": struct.unpack_from(">I", frame, 10)[0],
        "encoder": struct.unpack_from(">q", frame, 14)[0] if valid & 1 else "INVALID",
        "speed_mps": struct.unpack_from(">i", frame, 22)[0] / 10000.0 if valid & 2 else "INVALID",
        "steering_angle_deg": struct.unpack_from(">i", frame, 26)[0] / 10000.0 if valid & 4 else "INVALID",
        "road_wheel_deg": struct.unpack_from(">i", frame, 26)[0] / 10000.0 if (valid & 4 and not faults & 0x100000) else "INVALID",
        "legacy_speed_mapping": bool(faults & 0x80000),
        "legacy_eps_angle_mapping": bool(faults & 0x100000),
        "eps_actual_angle_count": struct.unpack_from(">i", frame, 30)[0] if valid & 8 else "INVALID",
        "eps_actual_angle_deg": float(struct.unpack_from(">i", frame, 30)[0]) if valid & 8 else "INVALID",
        "brake_mode": brake_mode,
        "brake_actual": brake_raw if valid & 16 else "INVALID",
        "brake_pressure_mpa": brake_raw / 100.0 if (valid & 16 and brake_mode == 1) else "INVALID",
        "brake_stroke_mm": brake_raw / 10.0 if (valid & 16 and brake_mode == 2) else "INVALID",
        "observed_drive_direction": struct.unpack_from("b", frame, 37)[0] if valid & 32 else "INVALID",
        "rt49_status": hex(struct.unpack_from(">H", frame, 38)[0]) if valid & 32 else "INVALID",
        "eps_status": hex(struct.unpack_from(">H", frame, 40)[0]) if valid & 64 else "INVALID",
        "seb_status": hex(struct.unpack_from(">H", frame, 42)[0]) if valid & 128 else "INVALID",
        "fault_flags": "0x{:08X}".format(faults),
        "last_received_cmd_counter": frame[48] if not faults & 0x40000 else "INVALID",
        "reserved": frame[49:52].hex(),
    }

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.5.11")
    parser.add_argument("--stm32", default="192.168.5.50")
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--log", help="Save accepted real UDP packets as JSONL (HEX and decoded fields)")
    args = parser.parse_args()
    with contextlib.ExitStack() as stack, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx:
        logfile = stack.enter_context(open(args.log, "w", encoding="utf-8")) if args.log else None
        rx.bind((args.bind, 5001))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
            control.bind((args.bind, 0))
            start = time.monotonic()
            end = start + args.duration
            next_probe = start
            next_print = start
            last_arrival = None
            last_counter = None
            intervals = []
            received = rejected = gaps = 0
            probe_counter = 0
            print("Listening on {}:5001, expecting STM32 {}:5001".format(args.bind, args.stm32))
            if args.probe:
                print("BENCH PROBE: disabled commands; existing EPS center output remains. Disconnect actuators.")
            while time.monotonic() < end:
                now = time.monotonic()
                if args.probe and now >= next_probe:
                    control.sendto(build_probe(probe_counter), (args.stm32, 5000))
                    probe_counter = (probe_counter + 1) & 255
                    next_probe = now + 0.02
                readable, _, _ = select.select([rx], [], [], 0.02)
                if not readable:
                    continue
                frame, sender = rx.recvfrom(2048)
                if sender != (args.stm32, 5001):
                    rejected += 1
                    continue
                try:
                    parsed = parse_status(frame)
                except ValueError as error:
                    rejected += 1
                    print(error)
                    continue
                arrival = time.monotonic()
                if logfile:
                    record = {"arrival_utc": datetime.datetime.utcnow().isoformat() + "Z",
                              "arrival_monotonic_s": arrival, "sender": list(sender),
                              "payload_hex": frame.hex(), "decoded": parsed}
                    logfile.write(json.dumps(record, ensure_ascii=False) + "\n")
                    logfile.flush()
                if last_arrival is not None:
                    intervals.append((arrival - last_arrival) * 1000)
                    delta = (parsed["counter"] - last_counter) & 255
                    if 1 < delta < 128:
                        gaps += delta - 1
                last_arrival = arrival
                last_counter = parsed["counter"]
                received += 1
                if arrival >= next_print:
                    print(parsed)
                    next_print = arrival + 1
            print("Received={}, rejected={}, counter gaps={}".format(received, rejected, gaps))
            if intervals:
                mean = sum(intervals) / len(intervals)
                print("Mean interval={:.2f}ms, observed rate={:.2f}Hz".format(mean, 1000 / mean))
            if not received:
                print("No status received: check NIC /24 address, cable, UDP5001 firewall and Keil debug counters.")

if __name__ == "__main__":
    main()
