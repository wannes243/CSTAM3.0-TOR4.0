"""Run the isolated portable regressions for this ROS package."""
from pathlib import Path
import subprocess
import sys


def test_package_regressions():
    root=Path(__file__).resolve().parents[4]
    subprocess.run([sys.executable,str(root/'tests/run_validation.py'),'--package','fabtino_bringup'],cwd=root,check=True)
