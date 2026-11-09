#!/usr/bin/env python3
"""Check Hall firmware build consistency; this is not an on-device boot test."""

import argparse
import hashlib
from pathlib import Path
import re
import struct
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def run(*args):
    return subprocess.check_output([str(arg) for arg in args], cwd=ROOT, text=True)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def motor_layout(readelf, obj):
    info = run(readelf, "--debug-dump=info", obj)
    matches = list(re.finditer(
        r"^\s*<(\d+)><([0-9a-f]+)>:.*?\((DW_TAG_\w+)\)", info, re.M))
    entries = [
        (int(match[1]), match[2], match[3],
         info[match.end():matches[i + 1].start() if i + 1 < len(matches) else len(info)])
        for i, match in enumerate(matches)
    ]
    typedef = next(entry for entry in entries if entry[2] == "DW_TAG_typedef" and
                   re.search(r"DW_AT_name\s*:.*\bmotor_all_state_t\b", entry[3]))
    reference = re.search(r"DW_AT_type\s*:\s*<0x([0-9a-f]+)>", typedef[3])[1]
    index = next(i for i, entry in enumerate(entries) if entry[1] == reference)
    structure = entries[index]
    size = int(re.search(r"DW_AT_byte_size\s*:\s*(\d+)", structure[3])[1])
    members = []
    for entry in entries[index + 1:]:
        if entry[0] <= structure[0]:
            break
        if entry[2] == "DW_TAG_member":
            name = next(line for line in entry[3].splitlines() if "DW_AT_name" in line)
            offset = re.search(r"DW_AT_data_member_location\s*:\s*(\d+)", entry[3])
            require(offset is not None, "Unsupported member offset in " + str(obj))
            members.append((name.rsplit(":", 1)[1].strip(), int(offset[1])))
    require(len(members) > 80, "Incomplete motor structure debug information")
    return size, members


def hex_binary(filename):
    memory = {}
    upper = 0
    eof = False
    for line in filename.read_text().splitlines():
        require(not eof and line.startswith(":"), "Invalid HEX record")
        record = bytes.fromhex(line[1:])
        require(len(record) >= 5 and len(record) == record[0] + 5 and
                sum(record) % 256 == 0, "Invalid HEX checksum or length")
        length, address, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        if kind == 4:
            require(length == 2, "Invalid HEX extended address")
            upper = int.from_bytes(record[4:6], "big") << 16
        elif kind == 0:
            for i, value in enumerate(record[4:4 + length]):
                location = upper + address + i
                require(location not in memory, "Overlapping HEX data")
                memory[location] = value
        elif kind == 1:
            eof = True
        else:
            require(kind == 5, "Unsupported HEX record type")
    require(eof and memory, "Incomplete HEX file")
    require(min(memory) == 0x08000000 and max(memory) < 0x08080000,
            "Unexpected firmware address range")
    require(not any(0x08004000 <= address < 0x0800C000 for address in memory),
            "HEX contains configuration-sector data")
    return bytes(memory.get(address, 0) for address in range(min(memory), max(memory) + 1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_dir", type=Path, help="Directory containing 60.elf/bin/hex and obj/")
    parser.add_argument("--prefix", default=str(
        ROOT / "tools/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-"))
    args = parser.parse_args()
    directory = args.build_dir.resolve()
    header = ROOT / "motor/foc_math.h"
    layouts = []
    for name in ("foc_math", "mcpwm_foc"):
        obj = directory / "obj" / (name + ".o")
        layouts.append(motor_layout(args.prefix + "readelf", obj))
    require(layouts[0] == layouts[1], "Motor structure size/member offsets disagree between objects")
    for name in ("foc_math", "mcpwm_foc"):
        obj = directory / "obj" / (name + ".o")
        require(obj.stat().st_mtime_ns >= max(header.stat().st_mtime_ns,
                (ROOT / "motor" / (name + ".c")).stat().st_mtime_ns),
                "Object is older than its source/header: " + str(obj))
        dependency = (directory / ".dep" / (name + ".o.d")).read_text()
        require(dependency.startswith(str(obj) + ":") and "motor/foc_math.h" in dependency,
                "Dependency file targets a different build or omits the Hall header")
    symbols = run(args.prefix + "nm", "-S", "--defined-only", directory / "obj/mcpwm_foc.o")
    allocation = re.search(r"^\S+\s+([0-9a-fA-F]+)\s+\w\s+m_motor_1$", symbols, re.M)
    require(allocation is not None and int(allocation[1], 16) == layouts[0][0],
            "Allocated motor state does not match its declared size")

    binary = (directory / "60.bin").read_bytes()
    stack, reset = struct.unpack_from("<II", binary)
    require(0x20000000 <= stack <= 0x20020000 and reset & 1 and
            0x08000000 <= reset < 0x08080000, "Invalid initial stack/reset vector")
    require(binary == hex_binary(directory / "60.hex"), "BIN and HEX differ")
    with tempfile.TemporaryDirectory(prefix="hall-firmware-check-") as temporary:
        extracted = Path(temporary) / "elf.bin"
        run(args.prefix + "objcopy", "-O", "binary", directory / "60.elf", extracted)
        require(binary == extracted.read_bytes(), "BIN and ELF differ")
    # Exercise the actual make dependency graph without changing any files.
    make_args = ("make", "-n", "-f", "make/fw.mk", "PROJECT=60",
                 "BUILDDIR=" + str(directory), "TCHAIN_PREFIX=" + args.prefix)
    up_to_date = run(*make_args)
    require(not re.search(r"echo (Compiling|Linking|Creating) ", up_to_date),
            "Build has pending compilation/linking; rebuild it before checking")
    dry_run = run(*make_args, "-W", "motor/foc_math.h")
    for name in ("foc_math", "mcpwm_foc"):
        require("echo Compiling " + name + ".c" in dry_run,
                "Header change does not rebuild " + name + ".c")
    print("PASS: motor layout matches (%d bytes, %d members)" %
          (layouts[0][0], len(layouts[0][1])))
    print("PASS: source freshness, dependencies, BIN/HEX/ELF consistency")
    print("BIN SHA256: " + hashlib.sha256(binary).hexdigest())
    print("On-device boot and motor behavior have not been verified by this check.")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, StopIteration, subprocess.CalledProcessError) as error:
        raise SystemExit("FAIL: " + str(error))
