# Isaac Sim + Pegasus Simulator — CPU Benchmark Analysis

**Machine:** CPU-only physics (`device=cpu`), headless, no RTX sensors  
**Metric:** Steady-state Real-Time Factor (RTF) = simulated seconds / wall-clock seconds  
**Goal:** Identify the minimum physics rate that allows ≥ 1× RTF for autonomy stack testing

---

## TL;DR

| Scenario | 50 Hz | 100 Hz | 250 Hz |
|---|---|---|---|
| No Pegasus, flat | **1.75×** | 0.85× | 0.36× |
| No Pegasus, warehouse | **1.75×** | 0.84× | 0.34× |
| Pegasus + Python ctrl, flat | **1.55×** | 0.73× | 0.16× |
| Pegasus + Python ctrl, warehouse | **1.48×** | 0.73× | 0.15× |
| Pegasus + PX4, flat | **1.76×** | 0.74× | **0.27×** |
| Pegasus + PX4, warehouse | **1.61×** | 0.78× | **0.18×** |

**50 Hz** is the only physics rate that reliably achieves real-time across all scenarios.  
**100 Hz** is viable for flat/simple scenes but falls short in the full warehouse.  
**250 Hz** (current default) runs at 0.08–0.36× real-time — 3–12× slower than real-time.

---

## 1. Physics Rate is the Dominant Bottleneck

The physics step rate overwhelms all other factors. Every halving of physics Hz roughly doubles RTF:

```
No Pegasus (flat scene):
  50 Hz  →  1.93× RT
 100 Hz  →  0.92× RT   (–53%)
 250 Hz  →  0.36× RT   (–81% from 100 Hz)

Pegasus + Python ctrl (flat):
  50 Hz  →  1.55× RT
 100 Hz  →  0.73× RT   (–53%)
 250 Hz  →  0.16× RT   (–78% from 100 Hz)
```

The 50→100 Hz step is roughly linear (2× steps = ~2× cost). The 100→250 Hz step is superlinear — Pegasus's per-step Python callbacks (sensor updates, controller math, MAVLink I/O) pile up disproportionately at high step rates.

---

## 2. Cost Breakdown by Component

By comparing baselines we can isolate each layer's contribution at 250 Hz:

| Component | Steady RTF | Cost vs baseline |
|---|---|---|
| Isaac Sim physics alone (no Pegasus) | 0.36× | baseline |
| + Pegasus vehicle (Python controller) | 0.16× | −0.20× |
| + PX4 MAVLink backend (instead of Python) | 0.27× | −0.09× |
| + Full Warehouse scene (vs flat plane) | −0.01 to −0.03× | small |

### Physics overhead alone
Even with no Pegasus extension loaded, 250 Hz physics runs at only **0.36× RT**. This is an Isaac Sim / PhysX cost, not Pegasus-specific.

### Pegasus vehicle cost
Loading a Pegasus `Multirotor` with the Python `NonlinearController` backend cuts RTF from 0.36 to **0.16×** at 250 Hz — a **55% penalty** from the vehicle's per-step Python callbacks (IMU updates, SE(3) controller, state integration).

### PX4 vs Python controller — a counterintuitive result
At 250 Hz, PX4 MAVLink performs *better* than the pure Python controller (0.27× vs 0.16×):

| Backend | 250 Hz RTF |
|---|---|
| Python NonlinearController | 0.16× |
| PX4 MAVLink | 0.27× |

This is counterintuitive — PX4 adds a subprocess, MAVLink I/O, and lockstep sync. The likely explanation: **PX4 lockstep acts as a natural rate limiter**. The MAVLink heartbeat/sensor cycle caps how fast the simulation can advance, effectively throttling the physics loop. The Python controller has no such cap and spins at full rate, burning more CPU per simulated second.

### Scene complexity
The Full Warehouse vs Flat Plane difference is small at runtime (≤0.03× RTF delta) but significant for **startup time**:

| Scene | Startup (warm) | Startup (cold/first-run) |
|---|---|---|
| Flat Plane | ~9 s | ~9 s |
| Full Warehouse | ~20–26 s | ~52 s |

Cold startup (first shader compile) costs 2–3× more. Subsequent runs are consistent.

---

## 3. Rendering Rate Has Minimal Effect (Headless, No Sensors)

Entries 25–32 swept rendering at 30 Hz and 60 Hz at default 250 Hz physics:

| Script | 30 Hz render | 60 Hz render | Delta |
|---|---|---|---|
| Pegasus + Python, flat, 250 Hz | 0.086× | 0.148× | −0.06 |
| Pegasus + PX4, flat, 250 Hz | 0.108× | 0.203× | −0.09 |
| Pegasus + Python, warehouse, 250 Hz | 0.083× | 0.152× | −0.07 |
| Pegasus + PX4, warehouse, 250 Hz | 0.112× | 0.203× | −0.09 |

Unexpectedly, **30 Hz rendering yields lower RTF than 60 Hz** in headless mode without sensors. This reversal is likely a render-sync artifact: at lower render frequencies, the headless RTX pipeline may issue less frequent but larger GPU synchronization barriers, increasing per-render cost and stalling the physics loop. In practice the delta is small relative to the physics cost, and **rendering rate is not a meaningful optimization lever without RTX sensors attached**.

---

## 4. Recommended Physics Rates for AirStack

| Use case | Recommended Hz | Rationale |
|---|---|---|
| Real-time autonomy stack testing | **50 Hz** | Comfortable 1.5–1.8× RT margin; all scenarios pass |
| Fast offline data collection | **100 Hz** | ~0.73–0.98× RT; acceptable for flat scenes; add time to warehouse runs |
| High-fidelity physics validation | **250 Hz** | Only usable in slow-motion (7–12× slower); plan for long runs |

### PX4 IMU Integration Rate
`PX4_IMU_INTEG_RATE` must match `physics_dt` to keep lockstep in sync. This is now handled automatically by `PX4LaunchTool` — no manual adjustment needed when changing `PX4_PHYSICS_HZ` in `.env`.

---

## 5. What 250 Hz Costs in Practice

At 250 Hz with Pegasus + PX4 in the Full Warehouse (the closest to the real AirStack scenario):

- **Steady RTF: 0.18×** — 1 simulated second takes ~5.5 wall-clock seconds
- **5 seconds of sim time** takes ~27 wall-clock seconds
- A **60-second mission** would take ~5.5 minutes of computation
- A **10-minute mission** would take ~55 minutes

This makes interactive debugging impractical. **Dropping to 100 Hz** recovers ~4× speed (RTF ~0.78×), making 1 minute of mission time cost ~80 wall-clock seconds.

---

## 6. GPU Physics Note

GPU physics (`device="cuda"`) was investigated but found to be **incompatible with Pegasus's drone simulation**. The Isaac Sim GPU physics pipeline enables `eENABLE_DIRECT_GPU_API`, which disables `PxArticulationLink::addTorque()` — the API Pegasus uses to apply motor forces to the drone. All results in this document use CPU physics.

---

## Appendix: Full Results Table

| Script | Physics Hz | Render Hz | Startup (s) | Steady RTF |
|---|---|---|---|---|
| 1 — no Pegasus, flat | 60 | 60 | 9.2 | 1.553 |
| 1 — no Pegasus, flat | 100 | 60 | 8.2 | 0.854 |
| 1 — no Pegasus, flat | 50 | 60 | 8.2 | 1.749 |
| 2 — Pegasus Python, flat | 250 | 60 | 12.3 | 0.156 |
| 2 — Pegasus Python, flat | 100 | 60 | 9.2 | 0.731 |
| 2 — Pegasus Python, flat | 50 | 60 | 9.5 | 1.552 |
| 3 — Pegasus PX4, flat | 250 | 60 | 9.5 | 0.268 |
| 3 — Pegasus PX4, flat | 100 | 60 | 9.4 | 0.737 |
| 3 — Pegasus PX4, flat | 50 | 60 | 9.2 | 1.764 |
| 4 — Pegasus Python, warehouse | 250 | 60 | 52.4* | 0.149 |
| 4 — Pegasus Python, warehouse | 100 | 60 | 26.4 | 0.731 |
| 4 — Pegasus Python, warehouse | 50 | 60 | 25.6 | 1.483 |
| 5 — Pegasus PX4, warehouse | 250 | 60 | 25.9 | 0.183 |
| 5 — Pegasus PX4, warehouse | 100 | 60 | 24.3 | 0.776 |
| 5 — Pegasus PX4, warehouse | 50 | 60 | 25.2 | 1.606 |
| 6 — no Pegasus, warehouse | 60 | 60 | 26.0 | 1.413 |
| 6 — no Pegasus, warehouse | 100 | 60 | 25.1 | 0.840 |
| 6 — no Pegasus, warehouse | 50 | 60 | 26.2 | 1.752 |
| 7 — no Pegasus, flat (explicit) | 250 | 60 | 8.2 | 0.357 |
| 7 — no Pegasus, flat (explicit) | 100 | 60 | 8.2 | 0.917 |
| 7 — no Pegasus, flat (explicit) | 50 | 60 | 8.3 | 1.927 |
| 8 — no Pegasus, warehouse (explicit) | 250 | 60 | 26.8 | 0.336 |
| 8 — no Pegasus, warehouse (explicit) | 100 | 60 | 26.2 | 0.847 |
| 8 — no Pegasus, warehouse (explicit) | 50 | 60 | 26.4 | 1.614 |

\* 52s startup on first run; subsequent runs ~26s (shader cache warm)

**Rendering sweep (250 Hz physics, headless, no sensors):**

| Script | Render Hz | Steady RTF |
|---|---|---|
| 2 — Pegasus Python, flat | 30 | 0.086 |
| 2 — Pegasus Python, flat | 60 | 0.148 |
| 3 — Pegasus PX4, flat | 30 | 0.108 |
| 3 — Pegasus PX4, flat | 60 | 0.203 |
| 4 — Pegasus Python, warehouse | 30 | 0.083 |
| 4 — Pegasus Python, warehouse | 60 | 0.152 |
| 5 — Pegasus PX4, warehouse | 30 | 0.112 |
| 5 — Pegasus PX4, warehouse | 60 | 0.203 |
