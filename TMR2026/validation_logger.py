"""CSV logging + scoring system for the "Sim2Real Validation of the
Autonomous Vehicle".

Produces the 3 deliverables (validation phase):
  - Test 1 (Latency):  P1_latency.csv
  - Test 2 (PID/STOP): P2_pid_stop.csv
  - Test 3 (FSM):      P3_fsm.csv

Plus a competition-style SCORING system that grades each test against its
acceptance criterion and produces a score report.

Usage:
    log = ValidationLogger()                 # creates validation_results/
    log.log_cycle(latency_ms, err, conf)     # every control cycle
    log.log_stop(dist_mm, pwm, err, state)   # while approaching the STOP
    log.log_fsm(new_state)                    # on every FSM transition
    log.save_all()                            # writes the 3 CSVs
    log.print_scoreboard()                    # prints score and saves report
"""

import os
import csv
import time
from typing import Optional


class ValidationLogger:

    LATENCY_MAX_MS      = 200.0
    LATENCY_BONUS_MS    = 100.0
    STOP_TARGET_MM      = 270.0
    STOP_TOLERANCE_MM   = 30.0
    FSM_EXPECTED_STOP    = ["CRUCERO", "PRECAUCION", "FRENADO", "ESPERA", "REANUDAR"]
    FSM_EXPECTED_PARKING = ["PARKING_SEARCH", "PARKING_MANEUVER", "PARKED"]

    def __init__(self, outdir: str = "validation_results"):
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.t0 = time.monotonic()

        self.latency_rows: list[dict] = []
        self.stop_rows:    list[dict] = []
        self.fsm_rows:     list[dict] = []

        self._last_fsm_state: Optional[str] = None
        self._last_fsm_time = self.t0


    def _t(self) -> float:
        return time.monotonic() - self.t0

    def log_cycle(self, latency_ms: float, error_px: float, conf: float) -> None:
        """Test 1: control-loop latency (perception -> response)."""
        self.latency_rows.append({
            "t_s":          round(self._t(), 4),
            "latency_ms":   round(latency_ms, 2),
            "error_px":     round(error_px, 1),
            "confidence":   round(conf, 3),
        })

    def log_stop(self, distance_mm: Optional[float], pwm: float,
                 error_px: float, fsm_state: str) -> None:
        """Test 2: braking at a STOP (ToF distance vs PWM vs time)."""
        self.stop_rows.append({
            "t_s":          round(self._t(), 4),
            "distance_mm":  round(distance_mm, 1) if distance_mm is not None else "",
            "pwm_duty":     round(pwm, 1),
            "error_px":     round(error_px, 1),
            "fsm_state":    fsm_state,
        })

    def log_fsm(self, new_state: str) -> None:
        """Test 3: record an FSM state transition."""
        if new_state == self._last_fsm_state:
            return
        now = self._t()
        self.fsm_rows.append({
            "t_s":          round(now, 4),
            "from_state":   self._last_fsm_state or "START",
            "to_state":     new_state,
            "dwell_prev_s": round(now - (self._last_fsm_time - self.t0), 3)
                            if self._last_fsm_state else 0.0,
        })
        self._last_fsm_state = new_state
        self._last_fsm_time = time.monotonic()


    def _write(self, name: str, rows: list[dict], fields: list[str]) -> str:
        path = os.path.join(self.outdir, name)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        return path

    def save_all(self) -> dict:
        paths = {}
        paths["latency"] = self._write(
            "P1_latency.csv", self.latency_rows,
            ["t_s", "latency_ms", "error_px", "confidence"])
        paths["pid_stop"] = self._write(
            "P2_pid_stop.csv", self.stop_rows,
            ["t_s", "distance_mm", "pwm_duty", "error_px", "fsm_state"])
        paths["fsm"] = self._write(
            "P3_fsm.csv", self.fsm_rows,
            ["t_s", "from_state", "to_state", "dwell_prev_s"])
        return paths


    def evaluate(self) -> dict:
        """Grade each test against its criterion and assign points (0-100)."""
        res = {"tests": [], "total": 0, "max": 0}

        lat = [r["latency_ms"] for r in self.latency_rows
               if 0 < r["latency_ms"] < 500]
        p1 = {"name": "P1 Control-loop latency",
              "points": 0, "max": 30, "detail": ""}
        if lat:
            avg = sum(lat) / len(lat)
            mx = max(lat)
            if avg < self.LATENCY_BONUS_MS:
                p1["points"] = 30
            elif avg < self.LATENCY_MAX_MS:
                p1["points"] = 20
            else:
                p1["points"] = 5
            p1["detail"] = (f"mean latency {avg:.1f} ms, max {mx:.1f} ms "
                            f"(target < {self.LATENCY_MAX_MS:.0f} ms)")
        else:
            p1["detail"] = "no latency data"
        res["tests"].append(p1)

        p2 = {"name": "P2 PID braking at STOP",
              "points": 0, "max": 40, "detail": ""}
        stopped = [r for r in self.stop_rows
                   if r["fsm_state"] in ("FRENADO", "ESPERA")
                   and isinstance(r["distance_mm"], (int, float))]
        if stopped:
            visible = [r["distance_mm"] for r in stopped
                       if r["distance_mm"] < 1000]
            if visible:
                s = sorted(visible)
                dist_final = s[len(s) // 2]
            else:
                dist_final = stopped[-1]["distance_mm"]
            err_dist = abs(dist_final - self.STOP_TARGET_MM)
            within = err_dist <= self.STOP_TOLERANCE_MM
            dists = [r["distance_mm"] for r in self.stop_rows
                     if r["fsm_state"] in ("PRECAUCION", "FRENADO", "ESPERA")
                     and isinstance(r["distance_mm"], (int, float))
                     and r["distance_mm"] < 1000]
            min_d = min(dists) if dists else dist_final
            overshoot = min_d < (self.STOP_TARGET_MM - 2.5 * self.STOP_TOLERANCE_MM)
            if within and not overshoot:
                p2["points"] = 40
            elif within:
                p2["points"] = 30
            elif err_dist <= 2 * self.STOP_TOLERANCE_MM:
                p2["points"] = 20
            else:
                p2["points"] = 5
            p2["detail"] = (f"stopped at {dist_final:.0f} mm "
                            f"(target {self.STOP_TARGET_MM:.0f}+/-{self.STOP_TOLERANCE_MM:.0f}), "
                            f"{'no' if not overshoot else 'with'} overshoot")
        else:
            p2["detail"] = "the car did not reach FRENADO/ESPERA (STOP not detected)"
        res["tests"].append(p2)

        p3 = {"name": "P3 FSM transitions",
              "points": 0, "max": 30, "detail": ""}
        visited = [r["to_state"] for r in self.fsm_rows]
        stop_cycle = [s for s in self.FSM_EXPECTED_STOP if s in visited]
        park_cycle = [s for s in self.FSM_EXPECTED_PARKING if s in visited]
        n_stop = len(stop_cycle)
        n_park = len(park_cycle)
        if n_stop >= 5 or n_park >= 3:
            p3["points"] = 30
        elif n_stop >= 3 or n_park >= 2:
            p3["points"] = 20
        elif n_stop >= 1 or n_park >= 1:
            p3["points"] = 10
        det = f"STOP {n_stop}/5: {', '.join(stop_cycle) or '-'}"
        if n_park > 0:
            det += f"  |  PARKING {n_park}/3: {', '.join(park_cycle)}"
        det += f"  ({len(self.fsm_rows)} transitions)"
        p3["detail"] = det
        res["tests"].append(p3)

        res["total"] = sum(p["points"] for p in res["tests"])
        res["max"]   = sum(p["max"] for p in res["tests"])
        return res

    def _board_text(self, res: dict) -> str:
        """Render the scoreboard (English)."""
        title = "SCOREBOARD - Sim2Real Validation TMR 2026"
        lines = ["=" * 60, "  " + title, "=" * 60]
        for p in res["tests"]:
            bar = "#" * int(20 * p["points"] / p["max"]) if p["max"] else ""
            lines.append(f"  {p['name']:<32} {p['points']:>3}/{p['max']:<3} {bar}")
            lines.append(f"     -> {p['detail']}")
        lines.append("-" * 60)
        pct = 100 * res["total"] / res["max"] if res["max"] else 0
        lines.append(f"  TOTAL: {res['total']}/{res['max']}  ({pct:.0f}%)")
        verdict = ("PASSED" if pct >= 70 else
                   "PARTIAL" if pct >= 40 else "REVIEW")
        lines.append(f"  VERDICT: {verdict}")
        lines.append("=" * 60)
        return "\n".join(lines)

    def print_scoreboard(self) -> dict:
        """Print the scoreboard and save SCOREBOARD.txt (English)."""
        res = self.evaluate()
        txt = self._board_text(res)
        print(txt)
        with open(os.path.join(self.outdir, "SCOREBOARD.txt"), "w",
                  encoding="utf-8") as f:
            f.write(txt + "\n")
        return res
