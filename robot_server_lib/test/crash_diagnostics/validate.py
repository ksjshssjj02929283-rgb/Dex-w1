#!/usr/bin/env python3
"""Standalone death tests; never starts ROS, accesses motors, or changes sysctls."""
import pathlib
import re
import resource
import signal
import subprocess
import sys
import tempfile

probe = pathlib.Path(sys.argv[1]).resolve()


def limit_child():
    # Per-process limits: allocation failure kills/fails ONLY the probe, not
    # unrelated desktop applications. No automatic core collector in death tests.
    resource.setrlimit(resource.RLIMIT_AS, (256 * 1024 * 1024, 256 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (5, 5))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


cases = {
    "segv": signal.SIGSEGV, "worker": signal.SIGSEGV,
    "unregistered": signal.SIGSEGV, "overflow": signal.SIGSEGV,
    "locked": signal.SIGSEGV, "abort": signal.SIGABRT,
    "exception": signal.SIGABRT, "bus": signal.SIGBUS,
    "ill": signal.SIGILL, "fpe": signal.SIGFPE,
    "term": signal.SIGTERM, "int": signal.SIGINT, "kill": signal.SIGKILL,
}
with tempfile.TemporaryDirectory(prefix="w1-crash-validation-") as root:
    for mode, expected in cases.items():
        directory = pathlib.Path(root) / mode
        result = subprocess.run([str(probe), mode, str(directory)],
                                capture_output=True, text=True, timeout=15,
                                preexec_fn=limit_child)
        assert result.returncode == -expected, (mode, result.returncode, result.stderr)
        logs = list(directory.glob("*.log"))
        assert len(logs) == 1, (mode, logs)
        assert logs[0].stat().st_mode & 0o777 == 0o600
        text = logs[0].read_text()
        assert "dumpable_before=0 dumpable=1" in text
        assert "core_pattern=" in text and "MAPS_BEGIN" in text
        records = re.findall(r"^CRASH .*", text, re.M)
        if mode in ("term", "int", "kill"):
            assert not records, (mode, records)
        else:
            assert len(records) == 1, (mode, records)
            record = records[0]
            assert f"signal={expected.value} " in record
            assert re.search(r"pc=0x[1-9a-f][0-9a-f]*", record), record
            tid = re.search(r" tid=(\d+)", record)[1]
            pid = re.search(r" pid=(\d+)", record)[1]
            if mode in ("worker", "unregistered", "overflow"):
                assert tid != pid
            if mode != "unregistered":
                registration = re.search(
                    rf"THREAD tid={tid} name=\S+ alt_mapping=(0x[0-9a-f]+) alt_mapping_size=(\d+)", text)
                assert registration, (mode, text)
                base, size = int(registration[1], 16), int(registration[2])
                stack = int(re.search(r"handler_stack_addr=(0x[0-9a-f]+)", record)[1], 16)
                assert base < stack < base + size, (mode, record)
        print(f"PASS {mode}: signal={expected.name}, crash_records={len(records)}")
