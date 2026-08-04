# version.py - PlatformIO pre-script: inject the firmware version at build time.
#
# The version has exactly one source, resolved in priority order:
#   1. $AMBIT_RELEASE_VERSION  - set by CI from the semantic-release tag
#   2. git describe            - dev builds ("v0.1.0-3-gabc123-dirty")
#   3. "0.0.0-dev"             - no git / no tags
#
# It feeds every reporting surface: the cmd 33/2 binary struct (Major/Minor/Batch),
# the text `hello` reply, and the openJII envelope identity. Before this script the
# tree carried four hand-maintained copies (nvs1.h, two literals in the JSON
# frontend, plus the Calibratron's out-of-repo pin) that had already diverged
# (0.0.6 vs 0.0.3 vs 0.0.5).
Import("env")

import os
import re
import subprocess


def _git_describe():
    try:
        return (
            subprocess.check_output(
                ["git", "describe", "--tags", "--always", "--dirty"],
                cwd=env["PROJECT_DIR"],
                stderr=subprocess.DEVNULL,
            )
            .decode()
            .strip()
        )
    except Exception:
        return ""


version = os.environ.get("AMBIT_RELEASE_VERSION", "").strip() or _git_describe() or "0.0.0-dev"
version = version.lstrip("vV")

# Major/Minor/Batch feed the frozen 48-byte cmd 33/2 wire struct (uint8 each).
m = re.match(r"^(\d+)\.(\d+)\.(\d+)", version)
major, minor, batch = (int(g) for g in m.groups()) if m else (0, 0, 0)

env.Append(
    CPPDEFINES=[
        ("AMBIT_FW_VERSION", env.StringifyMacro(version)),
        ("AMBIT_FW_MAJOR", major),
        ("AMBIT_FW_MINOR", minor),
        ("AMBIT_FW_BATCH", batch),
    ]
)
print("ambit fw version: %s (%d.%d.%d)" % (version, major, minor, batch))
