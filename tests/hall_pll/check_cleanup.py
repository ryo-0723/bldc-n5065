#!/usr/bin/env python3
"""Compare current control output to an archived source tree from before cleanup."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import tarfile
import tempfile

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument("baseline_archive", type=Path)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix="vesc-cleanup-check-") as temporary:
    work = Path(temporary)
    baseline = work / "baseline"
    baseline.mkdir()
    with tarfile.open(args.baseline_archive) as archive:
        archive.extractall(baseline, filter="data")
    outputs = []
    baseline_module = ("foc_speed_source.c" if (baseline / "motor/foc_speed_source.c").exists()
                       else "hall_edge_speed.c")
    for name, tree, module in [("baseline", baseline, baseline_module),
                               ("current", root, "foc_speed_source.c")]:
        executable = work / name
        if executable == baseline:
            executable = work / "baseline-test"
        command = ["gcc", "-O2", "-std=gnu99", "-Wall", "-Wextra", "-Werror",
                   "-ffunction-sections", "-fdata-sections"]
        command += ["-I" + str(path) for path in
                    [tree / "motor", root / "tests/hall_pll", root / "tests/utils_math",
                     root / "util", root]]
        command += [str(root / "tests/hall_pll/control_regression.c"),
                    str(tree / "motor/foc_math.c"), str(tree / "motor" / module),
                    str(root / "util/utils_math.c"), "-Wl,--gc-sections", "-lm", "-o", str(executable)]
        subprocess.run(command, check=True)
        outputs.append(subprocess.check_output([str(executable)]))
    if outputs[0] != outputs[1]:
        for index, (old, new) in enumerate(zip(outputs[0].splitlines(), outputs[1].splitlines())):
            if old != new:
                raise AssertionError(f"Control mismatch at sample {index}: {old!r} != {new!r}")
        raise AssertionError("Control replay lengths differ")
    print("PASS: pre-cleanup/current control outputs match bit-for-bit, 12 scenarios / 36000 PID samples")
    print("Replay SHA256:", hashlib.sha256(outputs[1]).hexdigest())
