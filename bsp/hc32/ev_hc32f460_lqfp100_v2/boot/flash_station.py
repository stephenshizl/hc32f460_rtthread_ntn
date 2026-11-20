# Copyright (c) 2026, youyoutech. SPDX-License-Identifier: Apache-2.0
"""Program and independently read back every HEX range on one SWD fixture."""
import argparse
from hashlib import sha256
from io import StringIO
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

from intelhex import IntelHex


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--sha256", required=True, help="Approved release digest")
    parser.add_argument("--probe", type=int, help="Fixture J-Link serial number")
    parser.add_argument("--jlink", default=os.environ.get("JLINK_EXE"))
    args = parser.parse_args()
    if not args.jlink:
        installed = (Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "SEGGER").glob("JLink*/JLink.exe")
        args.jlink = max(installed, key=lambda p: p.stat().st_mtime, default=None)
    if not args.jlink:
        parser.error("Set JLINK_EXE to the SEGGER JLink.exe path")
    data = args.image.read_bytes()
    if sha256(data).hexdigest() != args.sha256.lower():
        parser.error("Release HEX SHA-256 mismatch")
    image = IntelHex(StringIO(data.decode("ascii")))
    if not image.segments() or image.maxaddr() >= 0x7e000:
        parser.error("Outside L0 image area")
    with TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "release.hex").write_bytes(data)
        commands = ["r", "halt", "loadfile release.hex"]
        for index, (start, end) in enumerate(image.segments()):
            name = f"range{index}.bin"
            image.tobinfile(str(temp / name), start=start, end=end - 1)
            commands.append(f"verifybin {name} 0x{start:x}")
        (temp / "flash.jlink").write_text("\n".join(commands + ["r", "go", "exit"]) + "\n", encoding="ascii")
        cmd = [args.jlink, "-Device", "HC32F460xE", "-If", "SWD", "-Speed", "1000",
               "-ExitOnError", "1", "-NoGui", "1", "-CommandFile", "flash.jlink"]
        if args.probe:
            cmd.extend(["-USB", str(args.probe)])
        result = subprocess.run(cmd, cwd=temp, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=120)
        log = result.stdout.decode(errors="replace")
        print(log)
        if result.returncode or log.count("Verify successful.") != len(image.segments()):
            raise SystemExit("FAIL: incomplete J-Link readback")
        print(f"PASS {args.image.name} SHA256={args.sha256}")


if __name__ == "__main__":
    main()
