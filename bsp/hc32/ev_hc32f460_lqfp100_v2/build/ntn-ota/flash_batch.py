# Copyright (c) 2026, youyoutech. SPDX-License-Identifier: Apache-2.0
"""Program a rack of SWD fixtures from one approved release."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
from pathlib import Path
import re
import subprocess
import sys
from tempfile import mkdtemp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("fixtures", type=Path, help="CSV columns: unit,probe")
    parser.add_argument("--sha256", required=True)
    args = parser.parse_args()
    with args.fixtures.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or any(not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", r.get("unit") or "") or
                       not re.fullmatch(r"[1-9][0-9]{0,11}", r.get("probe") or "") for r in rows):
        parser.error("Each row needs a unit ID and a numeric J-Link serial number")
    for column in ("unit", "probe"):
        if len({r[column].lower() for r in rows}) != len(rows):
            parser.error(f"Duplicate {column}")
    directory = Path(mkdtemp(prefix="flash-log-", dir=Path.cwd()))

    def program(row):
        log = directory / ("unit-" + row["unit"] + ".log")
        cmd = [sys.executable, str(Path(__file__).with_name("flash_station.py")), str(args.image),
               "--sha256", args.sha256, "--probe", row["probe"]]
        with log.open("wb") as output:
            status = subprocess.run(cmd, stdout=output, stderr=subprocess.STDOUT).returncode
        print(f"{'FAIL' if status else 'PASS'} {row['unit']} probe={row['probe']} {log}", flush=True)
        return status

    with ThreadPoolExecutor(max_workers=min(16, len(rows))) as workers:
        results = list(workers.map(program, rows))
    raise SystemExit(int(any(results)))


if __name__ == "__main__":
    main()
