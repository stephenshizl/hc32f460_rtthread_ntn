# Copyright (c) 2026, youyoutech.
# SPDX-License-Identifier: Apache-2.0
"""Extract an address-checked MDK image or assemble a signed L0 provisioning HEX."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

from intelhex import IntelHex
from prepare_mdk import GEN, MCUBOOT, layout_int


def extract(path, base, limit):
    image = IntelHex(str(path))
    if image.minaddr() != base or image.maxaddr() >= base + limit:
        raise ValueError(f"{path}: load addresses outside [{base:#x}, {base + limit:#x})")
    return bytes(image.tobinarray(start=base, end=image.maxaddr()))


def check_vectors(image, base):
    stack, vector = struct.unpack_from("<II", image)
    if stack & 7 or not 0x1FFF8018 < stack <= 0x20027000:
        raise ValueError("Initial stack pointer outside main SRAM")
    if not vector & 1 or not base <= vector - 1 < base + len(image):
        raise ValueError("Reset vector outside the image")


def imgtool(*args):
    subprocess.run([sys.executable, str(MCUBOOT / "scripts" / "imgtool.py"), *map(str, args)], check=True)


def artifact(stage):
    return GEN / stage / f"ntn-ota-{stage}.hex"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("extract-bl2", "sign", "provision"))
    parser.add_argument("--key", type=Path, help="ECDSA P-256 signing key for development")
    parser.add_argument("--public-key", type=Path)
    parser.add_argument("--version", default="1.0.0")
    args = parser.parse_args()
    bl2_base = layout_int("NTN_OTA_BL2_BASE")
    bl2_size = layout_int("NTN_OTA_BL2_SIZE")
    primary = layout_int("NTN_OTA_PRIMARY")
    head = layout_int("NTN_OTA_HEADER_SIZE")
    slot = layout_int("NTN_OTA_SLOT_SIZE")
    if args.action == "extract-bl2":
        raw = extract(artifact("bl2"), bl2_base, bl2_size)
        check_vectors(raw, bl2_base)
        (GEN / "bl2.bin").write_bytes(raw)
        return
    if not args.key or not args.public_key:
        parser.error("sign/provision require --key and --public-key")

    app = extract(artifact("app"), primary + head, layout_int("NTN_OTA_APP_MAX"))
    check_vectors(app, primary + head)
    descriptor = layout_int("NTN_OTA_DESCRIPTOR_OFF")
    if app[descriptor:descriptor+4] != b"NTHC":
        raise ValueError("Missing App product descriptor at expected address")
    (GEN / "app-body.bin").write_bytes(app)
    compact = GEN / "app.signed.bin"
    sign_options = ("sign", "--key", args.key, "--version", args.version,
                    "--header-size", hex(head), "--pad-header", "--align", "4",
                    "--slot-size", hex(slot), "--max-sectors", "128")
    imgtool(*sign_options, GEN / "app-body.bin", compact)
    imgtool("verify", "--key", args.public_key, compact)
    if compact.stat().st_size > slot - layout_int("NTN_OTA_TRAILER_MAX"):
        raise ValueError("Signed App overlaps the trailer budget")
    if args.action == "sign":
        return

    bl1 = extract(artifact("bl1"), 0, layout_int("NTN_OTA_BL1_SIZE"))
    check_vectors(bl1, 0)
    cfg = layout_int("NTN_OTA_ICG_ADDR")
    icg = bl1[cfg:cfg+32]
    if len(icg) != 32 or icg[8:] != b"\xff" * 24:
        raise ValueError("L0 provisioning requires unprotected ICG4/5 and reserved words all ones")
    icg0, icg1 = struct.unpack_from("<II", icg, 0)
    # Expected L0 recipe: SWDT auto-start (PERIOD4096/DIV64 ~26.2s, count in sleep) + BOR LVL3 + HRC 16MHz.
    if icg0 != 0xFFDFEF66 or icg1 != 0xFFFBFEFF:
        raise ValueError(
            f"Unexpected BL1 ICG0/ICG1 recipe: got 0x{icg0:08X}/0x{icg1:08X}, "
            "expected 0xFFDFEF66/0xFFFBFEFF (did you pass --watchdog to prepare_mdk.py?)")
    bl2 = extract(artifact("bl2"), bl2_base, bl2_size)
    bl2 = bl2.ljust(bl2_size, b"\xff")
    digest = hashlib.sha256(bl2).digest()
    if digest not in bl1:
        raise ValueError("BL1 does not authorize this BL2")
    from cryptography.hazmat.primitives import serialization
    public = serialization.load_pem_public_key(args.public_key.read_bytes())
    der = public.public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo)
    if der not in bl2:
        raise ValueError("BL2 was built for a different App signing key")
    initial = GEN / "app.confirmed.bin"
    imgtool(*sign_options, "--pad", "--confirm", GEN / "app-body.bin", initial)
    if initial.stat().st_size != slot:
        raise ValueError("Unexpected primary slot length")
    flash = IntelHex()
    flash.frombytes(bl1, offset=0)
    flash.frombytes(bl2, offset=bl2_base)
    flash.frombytes(initial.read_bytes(), offset=primary)
    flash.write_hex_file(str(GEN / "ntn-ota-l0.hex"))
    factory = GEN / "factory.signed.bin"
    factory.write_bytes(compact.read_bytes())
    manifest = {
        "layout": layout_int("NTN_OTA_LAYOUT_ID"),
        "app_version": args.version,
        "public_key_sha256": hashlib.sha256(der).hexdigest(),
        "icg": icg.hex(),
        "onchip": [
            {"name": "bl1", "offset": 0, "size": len(bl1), "sha256": hashlib.sha256(bl1).hexdigest()},
            {"name": "bl2", "offset": bl2_base, "size": len(bl2), "sha256": digest.hex()},
            {"name": "primary", "offset": primary, "size": slot,
             "sha256": hashlib.sha256(initial.read_bytes()).hexdigest()},
        ],
        "nor": {"file": factory.name, "offset": layout_int("NTN_OTA_FACTORY"),
                "size": factory.stat().st_size, "sha256": hashlib.sha256(factory.read_bytes()).hexdigest()},
    }
    (GEN / "ntn-ota-l0.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="ascii")
    from flash_project import generate
    targets = {"ALL": "ntn-ota-l0.hex"}
    for entry in manifest["onchip"]:
        start = entry["offset"]
        name = entry["name"]
        filename = f"flash-{name}.hex"
        flash[start:start + entry["size"]].write_hex_file(str(GEN / filename))
        targets["APP" if name == "primary" else name.upper()] = filename
    generate(targets)


if __name__ == "__main__":
    main()
