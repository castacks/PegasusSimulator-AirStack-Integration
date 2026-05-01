# Pegasus Simulator — CPU Performance Benchmarks

Measures Isaac Sim real-time factor (RTF) across physics rates, scene complexities,
and drone backends to locate the source of CPU overhead in AirStack simulations.

**Headline metric:** `RTF = simulated seconds / wall-clock seconds`
RTF > 1 means the simulator runs faster than real time; RTF < 1 means slower.

See [ANALYSIS.md](ANALYSIS.md) for the full write-up and recommendations.

---

## Directory layout

```
benchmarking/
├── 1_cube_no_pegasus.py               # Isaac Sim only, no Pegasus, flat scene
├── 2_cube_pegasus_flat_no_px4.py      # Pegasus + Python controller, flat scene
├── 3_cube_pegasus_flat_px4.py         # Pegasus + PX4 MAVLink, flat scene
├── 4_cube_pegasus_complex_no_px4.py   # Pegasus + Python controller, Full Warehouse
├── 5_cube_pegasus_complex_px4.py      # Pegasus + PX4 MAVLink, Full Warehouse
├── 6_cube_no_pegasus_complex.py       # Isaac Sim only, no Pegasus, Full Warehouse
├── 7_cube_no_pegasus_250hz.py         # Isaac Sim only, explicit 250 Hz, flat
├── 8_cube_no_pegasus_complex_250hz.py # Isaac Sim only, explicit 250 Hz, Full Warehouse
├── run_all.py                         # Orchestrator: runs subsets, writes summary
├── utils/
│   ├── __init__.py
│   └── bench_timer.py                 # BenchTimer, BenchRuntime, shared helpers
├── results/
│   ├── <script_stem>.json             # Per-run metrics (one file per run)
│   ├── summary.json                   # Aggregated results written by run_all.py
│   ├── summary.png                    # 4-panel figure from run_all.py
│   └── analysis.png                   # Detailed 6-panel figure (see below)
└── README.md
```

---

## Scripts

### Core 8 (natural default rates)

| # | Script | Pegasus | Scene | Drone backend | Default physics Hz |
|---|--------|---------|-------|---------------|--------------------|
| 1 | `1_cube_no_pegasus.py` | No | default ground plane | — | 60 |
| 2 | `2_cube_pegasus_flat_no_px4.py` | Yes | Flat Plane | Python nonlinear ctrl | 250 |
| 3 | `3_cube_pegasus_flat_px4.py` | Yes | Flat Plane | PX4 MAVLink | 250 |
| 4 | `4_cube_pegasus_complex_no_px4.py` | Yes | Full Warehouse | Python nonlinear ctrl | 250 |
| 5 | `5_cube_pegasus_complex_px4.py` | Yes | Full Warehouse | PX4 MAVLink | 250 |
| 6 | `6_cube_no_pegasus_complex.py` | No | Full Warehouse | — | 60 |
| 7 | `7_cube_no_pegasus_250hz.py` | No | default ground plane | — | 250 |
| 8 | `8_cube_no_pegasus_complex_250hz.py` | No | Full Warehouse | — | 250 |

Scripts 7–8 accept `--physics-hz N` to benchmark at arbitrary rates.

### Extended suite (scripts 9–24, run via `run_all.py`)

Scripts 7, 8, 1, 6, 2, 4, 3, 5 re-run at 100 Hz and 50 Hz via `--physics-hz`:

| Range | Base scripts | Rate |
|-------|-------------|------|
| 9–10 | 7, 8 (no Pegasus, explicit rate) | 100 Hz |
| 11–12 | 7, 8 | 50 Hz |
| 13–14 | 1, 6 (no Pegasus, default) | 100 Hz |
| 15–16 | 1, 6 | 50 Hz |
| 17–18 | 2, 4 (Pegasus + Python) | 100 Hz |
| 19–20 | 2, 4 | 50 Hz |
| 21–22 | 3, 5 (Pegasus + PX4) | 100 Hz |
| 23–24 | 3, 5 | 50 Hz |

For PX4 scripts (21–24), `PX4_IMU_INTEG_RATE` is automatically set to match the
physics rate via `PegasusSimulator`'s `PX4LaunchTool`
(see `params.py` → `px4_mavlink_backend.py` → `px4_launch_tool.py`).

---

## Key pairing logic

| Comparison | What it isolates |
|-----------|-----------------|
| 1 vs 6 | Complex-scene cost at 60 Hz |
| 7 vs 8 | Complex-scene cost at 250 Hz |
| 1 vs 7, 6 vs 8 | Cost of 250 Hz physics alone (no Pegasus) |
| 7 vs 2, 8 vs 4 | Pegasus + drone overhead at matched physics_dt |
| 2 vs 3, 4 vs 5 | PX4 MAVLink cost vs Python controller |
| 7/9/11, 2/17/19, 3/21/23 | Physics rate sweep (250 → 100 → 50 Hz) |

---

## Running

### Inside the Isaac Sim container

```bash
# Single script (headless by default)
$ISAACSIM_PYTHON benchmarking/1_cube_no_pegasus.py

# Single script at a custom physics rate
$ISAACSIM_PYTHON benchmarking/7_cube_no_pegasus_250hz.py --physics-hz 100

# With display
$ISAACSIM_PYTHON benchmarking/1_cube_no_pegasus.py --no-headless
```

`$ISAACSIM_PYTHON` is set to `/isaac-sim/python.sh` in the Docker image.

### Run the full suite

```bash
# All 24 scripts — writes results/*.json + summary.json + summary.png
$ISAACSIM_PYTHON benchmarking/run_all.py

# Subset by number
$ISAACSIM_PYTHON benchmarking/run_all.py --scripts 1,3,5
$ISAACSIM_PYTHON benchmarking/run_all.py --scripts 1-8

# Re-plot without re-running (requires existing JSONs)
$ISAACSIM_PYTHON benchmarking/run_all.py --skip-run

# Custom output path
$ISAACSIM_PYTHON benchmarking/run_all.py --output /tmp/my_summary.png
```

### From outside the container

```bash
docker exec isaac-sim bash -c \
  "/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/extensions/PegasusSimulator/benchmarking/run_all.py"
```

---

## Output metrics

Each script writes `results/<script_stem>.json` containing:

| Field | Description |
|-------|-------------|
| `physics_dt`, `rendering_dt` | Actual rates used |
| `config` | `{pegasus, scene, drone_backend, headless}` |
| `startup_sim_app_s` | `SimulationApp(...)` construction time |
| `startup_world_and_scene_s` | World + scene + drone + cube + `reset()` |
| `startup_total_s` | Sum of the two above |
| `fall_rtf` | RTF during the cube fall phase (transient) |
| `steady_rtf` | RTF over 5 s post-landing **(headline metric)** |
| `rolling_rtf` | 1 s-window RTF samples during steady state (reveals jitter) |
| `landed`, `timed_out` | Whether the cube landed within the timeout |

`run_all.py` aggregates all results into `summary.json` and generates two figures:
- **`summary.png`** — 4-panel overview (startup breakdown, steady RTF, fall vs steady, jitter)
- **`analysis.png`** — 6-panel deep-dive (all 24 bars, RTF vs Hz curve, PX4 vs Python, scene cost, startup, rolling RTF)

---

## Summary of findings (from `results/summary.json`)

| Physics Hz | Config | Steady RTF |
|-----------|--------|-----------|
| 60 | No Pegasus, flat | **1.50** |
| 60 | No Pegasus, warehouse | **1.44** |
| 250 | No Pegasus, flat | 0.36 |
| 250 | No Pegasus, warehouse | 0.33 |
| 250 | Pegasus + Python, flat | 0.08 |
| 250 | Pegasus + PX4, flat | 0.10 |
| 100 | Pegasus + Python, flat | 0.41 |
| 100 | Pegasus + PX4, flat | 0.54 |
| 50 | Pegasus + Python, flat | **1.61** |
| 50 | Pegasus + PX4, flat | **1.72** |

**PX4 is not the bottleneck.** At every physics rate PX4 matches or slightly
*outperforms* the Python controller. The 250 Hz physics step rate is the primary
cause (~70% of slowdown); Pegasus per-step drone callbacks add the remaining ~30%.

At **50 Hz** all configurations run above real-time. At **100 Hz** Pegasus drops to
~0.4–0.57×. The `PX4_PHYSICS_HZ` constant in `params.py` is the primary tuning knob.

See [ANALYSIS.md](ANALYSIS.md) for the full causal breakdown and recommendations.

---

## Caveats

- First `SimulationApp` launch after a reboot includes shader / asset cache warm-up;
  run twice and use the second result.
- Only compare `headless`-to-`headless` or `--no-headless`-to-`--no-headless`; mixing adds noise.
- Scripts 3, 5, and 21–24 require the PX4 SITL binary at `pg.px4_path`
  (see `pegasus.simulator/config/configs.yaml`). They exit early if it is missing.
  After each PX4 run verify the process has exited: `ps aux | grep px4`.
- Each `run_all.py` invocation **overwrites** the per-script JSONs. Use `--skip-run`
  to re-plot without overwriting. Run the suite 3–5 times to assess variance.
