# Hall / PLL and Fast control tests

Specification: ../../docs/hall_pll_control.md.

- make run: source hysteresis and validation; actual PID/source integration; unequal-width Hall angle / legacy PLL; signed gain endpoints and handoff compensation.
- check_angles.py: fixed-input Hall angle trajectory against the repository HEAD.
- check_cleanup.py BEFORE_SOURCE_ARCHIVE: exact numerical comparison with the pre-cleanup source snapshot, 12 scenarios and 36000 PID samples. The snapshot is intentionally external to the source tree.
- check_firmware.py BUILD/60: source freshness, shared motor structure layout and BIN/HEX/ELF consistency.

These are host signal-processing tests and build checks. They do not prove real-motor stability, on-device boot or interrupt timing.
