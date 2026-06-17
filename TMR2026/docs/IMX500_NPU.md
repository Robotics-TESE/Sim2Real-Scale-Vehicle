# On-camera NPU detection (Sony IMX500)

The Pi AI Camera is more than a camera: the IMX500 sensor carries an **integrated
neural accelerator**. With the model loaded there, inference happens *inside the
camera* and the Pi receives, alongside every frame, the output tensors in the
metadata. The result: **detection at camera speed with ~0 % CPU**, leaving the 4
cores free for the lane, the FSM and the 50 Hz control loop.

## Fallback chain (there is no way to be left without a detector)

```
IMX500 NPU (.rpk)  ->  CPU NCNN  ->  CPU PyTorch (.pt)  ->  COLOR detector
     on-chip           ~25 ms          ~120 ms              STOP only
```

`main.py:_build_vision()` picks one: if `config.py:USE_IMX500_NPU=True` and
`weights/tmr_signs_imx500.rpk` exists, it uses the NPU; any failure falls back to
the CPU path **without interrupting startup**.

## Generating the .rpk (once, ON THE PI)

Sony's converter only exists on Linux, so this step is done on the Raspberry Pi
(unlike the NCNN export, which already ships in the repo).

```bash
# 1. Prerequisites (once)
sudo apt install -y imx500-all imx500-tools default-jre
pip3 install --break-system-packages model-compression-toolkit "imx500-converter[pt]"

# 2. Export (INT8 quantization calibrated with traffic_lights/)
cd ~/Carrito/TMR2026
python tools/export_imx500.py            # 15-60 min on the Pi 5 — once
```

The script leaves `weights/tmr_signs_imx500.rpk` + `weights/tmr_signs_imx500_labels.txt`
and on the next `python main.py` you will see:

```
[NPU] Loading model into the IMX500: weights/tmr_signs_imx500.rpk
[VISION] Backend: IMX500 NPU (on-chip inference)
```

> 💾 Optional: back up the .rpk to GitHub from the Pi —
> `git add weights/tmr_signs_imx500* && git commit -m "IMX500 rpk" && git push`.

## What changes and what does NOT change

| | CPU path (NCNN) | IMX500 NPU |
|---|---|---|
| Where the model runs | ARM CPU (thread at 15 Hz) | Inside the sensor (~30 Hz) |
| CPU used by detection | ~30-40 % of a core | ~0 % (only tensor parsing) |
| Accuracy | FP16 (= the .pt) | INT8 quantized (approx., calibrate conf) |
| FSM gating | only `stop_sign`/`red` brake | **identical** |
| 3-frame hysteresis | yes | **identical** |
| Color fallback (STOP) | yes | **identical** |
| Per-class pinhole distance | yes | **identical** |
| `LanePipeline` / PID / FSM | — | **unchanged** (same BGR frame) |

The FSM and `main.py` do not distinguish the backend: `IMX500CameraStream` exposes
the same API as `CameraStream` + `SignDetector` (`get_frame`, `get_detections`,
`has_sign`, `closest_sign`, ...).

## On-track calibration

- **`config.py:IMX500_CONF`** (default 0.55) — INT8 quantization can shift the
  confidences relative to the `.pt`. If the NPU does not see the sign from far,
  lower it toward 0.40; if it invents signs, raise it toward 0.65.
- Verify in **VISION** mode (`--display`): the bounding boxes and the object panel
  come out the same as on the CPU path.

## Going back to the CPU path

```python
# config.py
USE_IMX500_NPU = False
```
(or simply delete/rename the .rpk — the fallback is automatic).

## Common problems

| Symptom | Cause / fix |
|---|---|
| `imxconv-pt: command not found` | `pip3 install "imx500-converter[pt]"` |
| The converter asks for Java | `sudo apt install default-jre` |
| Export does not produce a .rpk | `imx500-tools` missing (apt) |
| `[VISION] NPU unavailable (...)` at startup | check the message — the system already continued on CPU; the car works the same |
| Strange confidences after quantizing | recalibrate `IMX500_CONF`; if not enough, re-export with `--fraction 1.0` |
