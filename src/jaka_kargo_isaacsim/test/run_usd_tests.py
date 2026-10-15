#!/usr/bin/env python3
"""Load Isaac's USD libraries without starting Kit, a GUI, or physics."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--isaac-root", type=Path, default=Path.home() / "isaacsim")
args = parser.parse_args()
root = args.isaac_root.expanduser()
usd = next((root / "extscache").glob("omni.usd.libs-*"))
physx = root / "extsPhysics/omni.usd.schema.physx"
env = os.environ.copy()
env["PYTHONPATH"] = os.pathsep.join([str(usd), str(physx), env.get("PYTHONPATH", "")])
env["LD_LIBRARY_PATH"] = os.pathsep.join([str(usd / "bin"), str(physx / "bin"), env.get("LD_LIBRARY_PATH", "")])
env["KARGO_PHYSX_PLUGIN_DIR"] = str(physx / "plugins/PhysxSchema/resources")
sys.exit(subprocess.call([sys.executable, "-m", "pytest", "-v", str(Path(__file__).with_name("test_usd_assembly.py"))], env=env))
