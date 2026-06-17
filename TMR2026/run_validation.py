"""Run the FULL validation in a single command.

Runs the 3 tests (latency, PID braking at STOP, FSM transitions), generates
the CSVs, computes the scoreboard and produces the article figures.

REQUIREMENT: Unity must be in PLAY (server listening on 127.0.0.1:5005).

Usage:
    python run_validation.py            # 60 s of validation
    python run_validation.py 90         # 90 s
"""

import sys
import socket
import subprocess
import os

PORT = 5005
DURATION = 60


def unity_listening(host="127.0.0.1", port=PORT, timeout=1.5) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        s.close()
        return True
    except Exception:
        return False


def main():
    dur = 50
    for a in sys.argv[1:]:
        try: dur = int(a)
        except ValueError: pass

    print("=" * 60)
    print("  SIM2REAL VALIDATION - TMR 2026 (3 tests)")
    print("=" * 60)

    if not unity_listening():
        print("\n[X] Unity is NOT listening on 127.0.0.1:5005.")
        print("    1) Open Unity and press PLAY.")
        print("    2) Check for 'Listening on port 5005' in the console.")
        print("    3) Run again:  python run_validation.py")
        sys.exit(1)
    print("[OK] Unity detected on port 5005.")
    print("[!] IMPORTANT: close any OTHER Python terminal connected")
    print("    (e.g. main_simulator.py --display). Unity serves only 1 client.\n")

    here = os.path.dirname(os.path.abspath(__file__))

    cmd = [sys.executable, os.path.join(here, "main_simulator.py"),
           "--validate", "--duration", str(dur), "--parking"]
    print(f">>> Running the FULL SEQUENCE for {dur} s...")
    print("    drives -> STOP (brake + wait 5s + resume) -> continues -> PARKS")
    print("    (covers the 3 tests in a single run)\n")
    subprocess.run(cmd, cwd=here)

    print("\n>>> Generating the article figures...")
    subprocess.run([sys.executable, os.path.join(here, "analyze_results.py"),
                    "validation_results"], cwd=here)

    print("\n" + "=" * 60)
    print("  VALIDATION COMPLETE")
    print("=" * 60)
    print("  Results in folder:  validation_results/")
    print("    - P1_latency.csv  P2_pid_stop.csv  P3_fsm.csv   (data)")
    print("    - fig1_latency.png  fig2_braking.png  fig3_fsm.png (figures)")
    print("    - SCOREBOARD.txt     (score report)")
    print("\n  Build the delivery package:  python armar_entrega.py")
    print("  (requirements map: docs/DELIVERY_PROFESSOR.md)")


if __name__ == "__main__":
    main()
