"""Build a trusted, local provisioning manifest for the A/B boot prototype."""

import argparse
import hashlib
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import zlib


def layout_value(name):
    header = Path(__file__).resolve().parents[1] / "firmware/ota/layout.h"
    match = re.search(rf"^#define {name} (0x[0-9a-fA-F]+)u$", header.read_text(), re.M)
    return int(match[1], 16)


def record(address, kind, payload):
    body = struct.pack(">BHB", len(payload), address, kind) + payload
    return ":" + (body + bytes([-sum(body) & 255])).hex().upper()


def sign_manifest(key, immutable_manifest, image):
    message = b"STM4OTA1" + immutable_manifest + hashlib.sha512(image).digest()
    with tempfile.NamedTemporaryFile() as source:
        source.write(message)
        source.flush()
        signature = subprocess.check_output([
            "openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(key),
            "-in", source.name,
        ])
    if len(signature) != 64:
        raise ValueError("expected an Ed25519 signature")
    return signature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("slot", choices=("a", "b"))
    parser.add_argument("version", type=int)
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--confirmed", action="store_true")
    parser.add_argument("--sign-key", type=Path)
    args = parser.parse_args()

    base = layout_value(f"OTA_SLOT_{args.slot.upper()}")
    meta = layout_value(f"OTA_META_{args.slot.upper()}")
    image = args.image.read_bytes()
    if not 8 <= len(image) <= layout_value("OTA_SLOT_SIZE"):
        parser.error("image size outside slot limits")
    if not 0 < args.version <= 0xFFFFFFFF:
        parser.error("version must fit in a nonzero uint32")
    stack, entry = struct.unpack_from("<II", image)
    if not (0x20000000 < stack <= 0x20020000 and stack % 8 == 0 and
            entry & 1 and base <= (entry & ~1) < base + len(image)):
        parser.error("invalid stack or entry vector for this slot")

    manifest = struct.pack("<7I", layout_value("OTA_MAGIC"), base, len(image),
                           zlib.crc32(image), args.version, 0xFFFFFFFF,
                           0 if args.confirmed else 0xFFFFFFFF)
    if args.sign_key:
        manifest += sign_manifest(args.sign_key, manifest[:20], image)
    lines = [record(0, 4, struct.pack(">H", meta >> 16))]
    for offset in range(0, len(manifest), 16):
        lines.append(record((meta & 65535) + offset, 0, manifest[offset:offset + 16]))
    lines.append(record(0, 1, b""))
    args.output.write_text("\n".join(lines) + "\n", encoding="ascii")


if __name__ == "__main__":
    main()
