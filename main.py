"""Root loader for the TMR 2026 vehicle.

This file exists only so the user can run from the repository root:

    python main.py [--display]

It delegates all logic to TMR2026/main.py, preserving the CWD and relative
imports (vision/, hardware/, control/, autonomy/).

The systemd service (TMR2026/systemd/carrito_tmr.service) still points
directly to TMR2026/main.py -- this loader is ONLY for manual execution.
"""
import os
import sys
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))
TMR  = os.path.join(HERE, "TMR2026")

if not os.path.isdir(TMR):
    sys.exit(f"[ERROR] TMR2026 folder not found in {HERE}")

os.chdir(TMR)
sys.path.insert(0, TMR)

runpy.run_path(os.path.join(TMR, "main.py"), run_name="__main__")
