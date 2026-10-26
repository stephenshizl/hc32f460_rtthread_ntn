"""Check the real ARMCC call graph, including PC router callback edges.

Link with --callgraph --callgraph_file=<path>, then run:
    python tests/test_pc_at_stack.py <path>

The linker cannot connect the RS232 RX callback or maintenance command
callback automatically. Stitch those edges explicitly; host GCC tests
cannot reproduce the target's RTOS thread stack exhaustion.
"""
import argparse
import html
from pathlib import Path
import re

BSP = Path(__file__).resolve().parents[1]
# Allow for indirect driver callbacks and RTOS context not included below.
HEADROOM = 1024


def stack_record(report, name):
    match = re.search(
        r"<P><STRONG>[^\n]*>" + re.escape(name) +
        r"</STRONG>.*?(?=<P><STRONG>|\Z)", report, re.S)
    if match is None:
        raise AssertionError(f"Missing linked function: {name}")
    record = html.unescape(re.sub(r"<[^>]+>", " ", match[0]))
    frame = re.search(r"Stack size (\d+) bytes", record)
    depth = re.search(r"Max Depth = (\d+)", record)
    if frame is None or depth is None:
        raise AssertionError(f"Missing stack data: {name}")
    return int(frame[1]), int(depth[1])


def check(report):
    source = (BSP / "applications/pc_rs232.c").read_text(encoding="utf-8")
    capacity = int(re.search(
        r"#define\s+RS232_THREAD_STACK_SZ\s+(\d+)", source)[1])
    # Both callbacks execute synchronously on rs232_rx, not on a worker.
    outer = sum(stack_record(report, name)[0] for name in (
        "rs232_rx_thread_entry", "rs232_dispatch_chunk", "terminal_rs232_rx"))
    paths = {
        "mode0 local AT/NV": outer + stack_record(
            report, "ntn_bridge_process_pc_data")[1],
        "mode1 maintenance AT/NV": outer + stack_record(
            report, "pc_maint_input")[0] + stack_record(
                report, "maint_local_command")[1],
    }
    for name, used in paths.items():
        required = used + HEADROOM
        print(f"{name}: linked={used}, reserve={HEADROOM}, "
              f"required={required}, allocated={capacity} bytes")
        assert capacity >= required, f"rs232_rx stack too small for {name}"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("callgraph", type=Path)
    args = parser.parse_args()
    check(args.callgraph.read_text(encoding="utf-8", errors="replace"))
