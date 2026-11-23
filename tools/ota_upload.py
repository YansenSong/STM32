"""Stage a signed STM32 image over UDP in a locally armed maintenance session."""

import argparse
from pathlib import Path
import socket
import struct
import time
import zlib

from ota_manifest import layout_value

PORT = 5003
HEADER = struct.Struct("<4sBBHI")
CHUNK = 256
BEGIN, DATA, FINISH, REBOOT = range(1, 5)


def read_manifest(path):
    memory = {}
    upper = 0
    for line in path.read_text().splitlines():
        raw = bytes.fromhex(line[1:])
        length, address, kind = struct.unpack_from(">BHB", raw)
        if not line.startswith(":") or len(raw) != length + 5 or sum(raw) & 255:
            raise ValueError("invalid Intel HEX record")
        data = raw[4:-1]
        if kind == 4 and length == 2:
            upper = int.from_bytes(data, "big") << 16
        elif kind == 0:
            for offset, byte in enumerate(data):
                absolute = upper + address + offset
                if absolute in memory:
                    raise ValueError("overlapping manifest records")
                memory[absolute] = byte
        elif kind == 1:
            break
        else:
            raise ValueError("unsupported Intel HEX record")
    start = min(memory, default=0)
    if start not in (layout_value("OTA_META_A"), layout_value("OTA_META_B")) or len(memory) != 92:
        raise ValueError("expected a signed slot manifest")
    return bytes(memory[start + i] for i in range(92))


def exchange(sock, op, nonce, offset=0, body=b"", timeout=1.0):
    packet = HEADER.pack(b"OTA1", op, 0, nonce, offset) + body
    for _ in range(5):
        sock.send(packet)
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            sock.settimeout(remaining)
            try:
                reply = sock.recv(128)
            except socket.timeout:
                break
            if len(reply) != HEADER.size:
                continue
            magic, reply_op, status, reply_nonce, received = HEADER.unpack(reply)
            if (magic, reply_op, reply_nonce) != (b"OTA1", op, nonce):
                continue
            if status:
                raise RuntimeError(f"board rejected operation {op}: status={status}, offset={received}")
            return received
    raise TimeoutError(f"no acknowledgement for operation {op}, offset={offset}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--bind", default="192.168.5.11")
    parser.add_argument("--board", default="192.168.5.50")
    parser.add_argument("--reboot", action="store_true")
    args = parser.parse_args()
    try:
        image = args.image.read_bytes()
        manifest = read_manifest(args.manifest)
        magic, base, length, crc, version, attempted, confirmed = struct.unpack_from("<7I", manifest)
        if (magic != layout_value("OTA_MAGIC") or
                base not in (layout_value("OTA_SLOT_A"), layout_value("OTA_SLOT_B")) or
                length != len(image) or crc != zlib.crc32(image) or
                attempted != 0xFFFFFFFF or confirmed != 0xFFFFFFFF):
            raise ValueError("image and trial manifest do not match")

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.bind, PORT))
            sock.connect((args.board, PORT))
            exchange(sock, BEGIN, 1, body=manifest, timeout=12.0)
            for i, offset in enumerate(range(0, length, CHUNK), start=2):
                data = image[offset:offset + CHUNK]
                if exchange(sock, DATA, i, offset, data) != offset + len(data):
                    raise RuntimeError("board acknowledged an unexpected offset")
            if exchange(sock, FINISH, i + 1) != length:
                raise RuntimeError("board finalized an unexpected image length")
            print(f"Staged version {version}, {length} bytes at 0x{base:08x}")
            if args.reboot:
                sock.send(HEADER.pack(b"OTA1", REBOOT, 0, i + 2, 0))
                print("Requested reboot; check which version the signed bootloader starts.")
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        parser.exit(1, f"OTA failed: {exc}\n")


if __name__ == "__main__":
    main()
