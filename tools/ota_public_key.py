"""Generate a C trust anchor from an Ed25519 public PEM key."""

import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("key", type=Path)
    parser.add_argument("header", type=Path)
    parser.add_argument("--match-private", type=Path)
    args = parser.parse_args()
    der = subprocess.check_output([
        "openssl", "pkey", "-pubin", "-in", str(args.key), "-outform", "DER",
    ])
    prefix = bytes.fromhex("302a300506032b6570032100")
    if len(der) != len(prefix) + 32 or not der.startswith(prefix):
        parser.error("expected an Ed25519 public key")
    if args.match_private:
        derived = subprocess.check_output([
            "openssl", "pkey", "-in", str(args.match_private), "-pubout", "-outform", "DER",
        ])
        if derived != der:
            parser.error("public key does not match signing key")
    values = ", ".join(f"0x{value:02x}" for value in der[len(prefix):])
    args.header.write_text(f"static const unsigned char ota_public_key[32] = {{ {values} }};\n")


if __name__ == "__main__":
    main()
