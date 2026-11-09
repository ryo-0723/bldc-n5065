#!/usr/bin/env python3
import subprocess,tempfile
from pathlib import Path
r=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="hall-angle-check-") as t:
    t=Path(t);baseline=t/"baseline.c";baseline.write_bytes(subprocess.check_output(["git","show","HEAD:motor/foc_math.c"],cwd=r))
    outputs=[]
    for name,source in [("baseline",baseline),("current",r/"motor/foc_math.c")]:
        exe=t/name
        subprocess.run(["gcc","-O2","-std=gnu99","-Wall","-Wextra","-Werror","-ffunction-sections","-fdata-sections",
            "-I"+str(r/"tests/hall_pll"),"-I"+str(r/"tests/utils_math"),"-I"+str(r/"motor"),"-I"+str(r/"util"),"-I"+str(r),
            str(r/"tests/hall_pll/angle_regression.c"),str(source),str(r/"motor/foc_speed_source.c"),str(r/"util/utils_math.c"),
            "-Wl,--gc-sections","-lm","-o",str(exe)],check=True)
        outputs.append(subprocess.check_output([str(exe)]))
    assert outputs[0]==outputs[1],"Existing Hall angle trajectory changed"
    print("PASS: existing Hall angle trajectory is bit-for-bit unchanged (forward/reverse, invalid readings, sensorless transition)")
