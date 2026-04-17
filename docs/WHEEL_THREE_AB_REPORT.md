# Wheel Status Report

This is the canonical wheel status document for the current codebase.

- Current code-path status
- current experiment interpretation
- current next-step plan

Other wheel markdown files in this directory should be treated as historical reference unless explicitly updated to match this file.

## Current Status (2026-04-17)

This section is the current handoff summary. Read this first.

### 0b. Latest direct A/B: `full_wheel_graph` vs `pure_imu` (5 runs)

Batch: `ORB_SLAM3_ROS2/experiments/ab_batch_full_vs_pure_20260417_1613`
(`s100p_mono_imu_fullwheel.yaml`, script-internal `wheel_on` vs `wheel_off`).

- Aggregate from `batch.log`:
    - `wheel_on` (full wheel): `ratio_mean=0.8770`, `ratio_std=0.0968`,
      `gap_mean=0.6961`, `gap_std=0.5492`
    - `wheel_off` (pure IMU): `ratio_mean=0.8441`, `ratio_std=0.0925`,
      `gap_mean=0.9136`, `gap_std=0.3039`
- Direct delta (`wheel_on - wheel_off`):
    - ratio: `+0.0329` (about +3.9% relative)
    - loop_gap: `-0.2175` (about -23.8%, lower is better)
- Run-level behavior from `summary.tsv`:
    - ratio: wheel_on wins 3/5 runs
    - loop_gap: wheel_on wins 3/5 runs
    - one outlier run (`wheel_on` gap `1.5699`) inflates variance.

Interpretation:
- The previously open question "does full wheel help compared with pure IMU?"
  is now answered as **yes, with positive trend but limited robustness**.
- Wheel path is effective, but run-to-run variance is still high; this is not
  yet a strong/stable win under strict acceptance gates.

### 0. P0 Scheme A (wheel tail residual) - contract locked in code

Status: P0-A code changes landed, Jacobian self-check passed, and three-mode
A/B has been re-run (2026-04-17).

- Contract (MUST stay stable; any future change requires rederivation of the
  tail Jacobian in `EdgeInertialGSE::linearizeOplus`):
    - Preintegration state (12D in `covariance_enc.block<12,12>(0,0)`):
      `[delta_phi(3), delta_v(3), delta_p(3), delta_enc_v(3)]`, all right-
      perturbation, all expressed in `{b1}` (the body frame at the beginning
      of the window).
    - Wheel accumulator: `encoder_velocity ~= R_b1_w * (t_wo2 - t_wo1)` with
      `t_wo_k = t_wb_k + R_wb_k * Tbo`. Scale `s` is NOT applied here.
    - Tail residual:
      `e_tail = Rbw1 * (s * dpb + Rwb2 * Tbo) - Tbo - enc_v`,
      `dpb = twb2 - twb1`. Scale `s` acts only on `dpb`, never on the lever
      arm `Rwb2*Tbo - Tbo`.

- Code touch-points (algorithm unchanged, only locked-down + documented):
    - `ORB_SLAM3/src/ImuTypes.cc` - contract header above the wheel-enabled
      `IntegrateNewMeasurement`, per-block comments on `F` and `Vmat`, known
      limitation (Nga_en injects nv^2 on y/z too) flagged inline.
    - `ORB_SLAM3/src/G2oTypes.cc`
        - contract comment above `ComputeWheelTailResidual`;
        - tail Jacobian block derivations (dphi1/dt1/dphi2/dt2/s/bg) written
          as comments next to each assignment in `linearizeOplus`;
        - dead `// std::cout` lines removed from `computeError`;
        - `[WheelJacCheck] tail jac mismatch` log now also prints `s`,
          `||dpb||`, `||enc_v||`.
    - `ORB_SLAM3/CMakeLists.txt` - added
      `option(ORB_SLAM3_WHEEL_TAIL_JAC_CHECK ... OFF)` that wires into
      `add_compile_definitions(ORB_SLAM3_WHEEL_TAIL_JAC_CHECK)`.

- Jacobian self-check status (2026-04-17): PASS.
    - Build: `RelWithDebInfo -DORB_SLAM3_WHEEL_TAIL_JAC_CHECK=ON` (Debug
      build is blocked by an unrelated pre-existing assertion at
      `Optimizer.cc:2837` in `LocalInertialBA`; RelWithDebInfo keeps symbols
      but disables `assert` via `NDEBUG`).
    - Smoke run: one pass of `s100p_mono_imu_fullwheel.yaml` on
      `large_single_0.db3`; the bag played to completion, atlas saved,
      `KeyFrameTrajectory.txt` written.
    - Result: `grep -c '\[WheelJacCheck\]' run.log == 0` on the full run,
      i.e. analytic tail Jacobian matches the finite-difference numeric
      Jacobian within 1e-4 across every `EdgeInertialGSE::linearizeOplus`
      invocation. The P0 Scheme A contract is code-level consistent.

- Unrelated issues observed during the smoke run (not in scope for P0-A,
  logged here for later triage):
    - `Optimizer.cc:2837 assert(mit->second>=3)` aborts any Debug build
      once `LocalInertialBA` encounters a local KF with fewer than 3
      visual observations. Reproduces without the wheel path.
    - First-round `[VIBA] begin bg_before=[ 4.6e170 ...]` printed garbage:
      `LocalMapping::InitializeIMU` first branch (`!isImuInitialized()`)
      sets only `mRwg`, leaving `mbg`/`mba` uninitialized before the log
      line. Cosmetic only; `InertialOptimization` reads biases from KFs,
      not from `mbg`. One-line fix candidate for a later cleanup.

### P0 Scheme A result (2026-04-17, batch `ab_batch_p0A_20260417`)

- Data source:
    - `preint_vs_pure/wheel_off_*` -> `pure_imu_clean`
    - `preint_vs_pure/wheel_on_*` -> `wheel_preint_only`
    - `fullgraph_vs_pure/wheel_on_*` -> `full_wheel_graph`
- Evaluation command: `scripts/eval_traj_vs_wheel.py` on 9 trajectories
  (3 modes x 3 runs), wheel reference from
  `/home/xzb/Desktop/dataset/ros2_bag/large_single/large_single_0.db3`.

- Per-mode aggregate (from `eval_three_mode.log`):
    - `pure_imu_clean`: ratio mean/std = `0.8484 / 0.1042`,
      loop_gap mean/std = `0.7305 / 0.4273`
    - `wheel_preint_only`: ratio mean/std = `0.9114 / 0.0457`,
      loop_gap mean/std = `0.6814 / 0.4145`
    - `full_wheel_graph`: ratio mean/std = `0.9425 / 0.0349`,
      loop_gap mean/std = `0.3489 / 0.0536`

- Direct effect vs pure IMU (`full_wheel_graph` - `pure_imu_clean`):
    - ratio: `+0.0941` (about +11% relative)
    - loop_gap: `-0.3816` (about -52% relative)
  -> Wheel path is clearly active and brings positive movement on these two
  metrics in this batch.

- Acceptance check (thresholds from `WHEEL_RISK_EVALUATION.md` s8, three-mode
  restatement):
    - `full_wheel_graph` vs `pure_imu_clean`:
      - ratio mean >= 0.90: PASS (`0.9425`)
      - ratio gap <= 3%: FAIL (mean gap about `+9.4%`)
      - loop_gap not worse than `pure_imu_mean + 1sigma`: PASS
      - reset/init-fail not increased: pending explicit count extraction
    - `full_wheel_graph` vs `wheel_preint_only`:
      - at least one metric improves by >= 1sigma: FAIL

- Verdict:
    - Under the current strict acceptance gate, P0 Scheme A is **not accepted**.
    - Under the practical question "is wheel helping compared with pure IMU?",
      answer is **yes** on this batch.

- Next step recommendation:
    1. Keep runtime rollback switch (`Wheel.UseVIBAEdge`) as-is.
    2. Extract reset/init-fail counts to complete the remaining acceptance row.
    3. Enter P1/P2 discussion for robust gain under strict thresholds, or
       revisit threshold definition if the project objective is "measurable
       improvement vs pure IMU" rather than "near-equality vs pure IMU ratio".


### 1. What has been confirmed

- `wheel_off` is now confirmed to be a clean `pure IMU` baseline.
  - Input layer: when `use_wheel=false`, the ROS2 node does not subscribe/feed wheel data.
  - Core preintegration layer: `System(..., useWheel_)` passes `false`, so `Tracking` does not call `SetWheel(true, ...)`, and `mpImuCalib->mbUseWheel` stays false.
  - Graph optimization layer: `EdgeInertialGSE` is only created when both `UseWheelEncoder()` and `UseWheelEncoderVIBA()` are true; for `wheel_off`, this is false.

- The current project no longer uses `Wheel.use` as the primary experiment switch.
  - The real runtime master switch is `use_wheel`.
  - The VIBA graph-side switch is `Wheel.UseVIBAEdge`.

- Current `VIBA` behavior is:
  - `use_wheel=false`: pure IMU
  - `use_wheel=true` and `Wheel.UseVIBAEdge=0`: wheel preintegration is on, but VIBA does not insert `EdgeInertialGSE`
  - `use_wheel=true` and `Wheel.UseVIBAEdge=1`: full wheel path, including VIBA wheel edge

### 2. What the recent 3-run A/B actually measured

The recent batch under `ab_batch_04170.02` was **not** `pure IMU vs full wheel`.

It was:

- `wheel_off`: `pure IMU`
- `wheel_on`: `wheel preintegration only`

because the config used:

- `use_wheel=true`
- `Wheel.UseVIBAEdge: 0`

So this batch only answers:

> Does enabling wheel-related preintegration logic, without VIBA wheel graph edges, show a stable gain?

Current answer:

- No stable net gain is visible yet.
- New summary fields added to `scripts/analyze_ab_results.py` show:
  - `scale_after`: almost no meaningful separation
  - `dbg_norm`: wheel-on is slightly larger
  - `dba_norm`: wheel-on is slightly smaller
  - all differences are small and not convincing at `n=3`

### 3. Current interpretation

- The previous narrow question, "should VIBA keep wheel edge enabled?", has basically been answered:
  - there is no evidence yet that keeping the VIBA wheel edge gives a stable benefit
  - disabling it did not produce a clear regression in the current small A/B

- The larger question, "does full wheel fusion have engineering value?", is now **partially answered**.
  - We have run clean `pure IMU vs full wheel graph-on` comparison (5 runs).
  - Result shows measurable average improvement vs pure IMU, but robustness is
    not yet strong enough to claim stable production-grade gain.

### 4. Files that matter right now

- Runtime switch and wheel input path:
  - `ORB_SLAM3_ROS2/src/monocular-inertial/mono-inertial-node.cpp`
  - `ORB_SLAM3_ROS2/run_orb_experiment.sh`

- Core wheel enable path:
  - `ORB_SLAM3/src/Tracking.cc`
  - `ORB_SLAM3/include/ImuTypes.h`

- VIBA wheel edge path:
  - `ORB_SLAM3/src/LocalMapping.cc`
  - `ORB_SLAM3/src/Optimizer.cc`

- Current default config (`preintegration-only` when `use_wheel=true`):
  - `ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final.yaml`

- New full-wheel config (`graph-on`):
  - `ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_fullwheel.yaml`

### 5. Immediate next step

Immediate work items (after latest 5-run full-vs-pure batch):

1. Extract run-level stability counters from all new runs:
   - `reset_count`
   - `init_fail_count`
   - `bad_loop_count`
   and correlate them with ratio/loop_gap outliers.
2. Keep 3-mode framing as the canonical analysis view:
   - `pure_imu_clean`
   - `wheel_preint_only`
   - `full_wheel_graph`
3. Prepare P1/P2 stabilization options, while keeping runtime rollback switch
   (`Wheel.UseVIBAEdge`) enabled.

Historical plan (kept for traceability):

Run and compare these three modes explicitly:

1. `pure_imu_clean`
   - config: `s100p_mono_imu_final.yaml`
   - runtime: `use_wheel=false`

2. `wheel_preint_only`
   - config: `s100p_mono_imu_final.yaml`
   - runtime: `use_wheel=true`
   - note: `Wheel.UseVIBAEdge: 0`

3. `full_wheel_graph`
   - config: `s100p_mono_imu_fullwheel.yaml`
   - runtime: `use_wheel=true`
   - note: `Wheel.UseVIBAEdge: 1`

Use these as the main comparison metrics:

- `ratio`
- `loop_gap`
- `scale_after`
- `dbg_norm`
- `dba_norm`
- reset / initialization failure count

Treat `loop_detected` and `bad_loop` as secondary side-effect signals only.

### 6. Important caution

Some historical sections below were written under older assumptions, especially around `Wheel.use` and older wheel-edge behavior. They are still useful as background, but the current code-path summary above should be treated as the source of truth for the next agent step.

## Historical Code-Side Findings

These findings remain useful, but they should now be read as:

- graph-on risk analysis
- or earlier-code-path observations

They are **not** automatically equivalent to the current default runtime path, because the current default config keeps `Wheel.UseVIBAEdge: 0`.

### P1. Inertial initialization fixes poses but still applies wheel tail residual
- File: `ORB_SLAM3/src/Optimizer.cc`
- In `InertialOptimization()`, `VertexPose` is fixed while `EdgeInertialGSE` still uses a tail residual that depends on `Rwb1`, `Rwb2`, `twb1`, `twb2`, and `s`.
- Risk: the optimizer can only push `scale`, `velocity`, `bg`, and `ba` to absorb wheel inconsistency instead of correcting pose geometry.

### P2. Wheel tail residual is sensitive to scale and extrinsic assumptions
- File: `ORB_SLAM3/src/G2oTypes.cc`
- `ComputeWheelTailResidual()` uses:
  - `Rbw1 * (s * dpb + Rwb2 * Tbo) - Tbo - enc_v`
- Risk: if the scale convention, `Tbo`, or `encoder_velocity` frame/units are even slightly inconsistent, the result appears as a persistent ratio bias.

### P3. Wheel covariance/information weighting still looks fragile
- Files: `ORB_SLAM3/src/ImuTypes.cc`, `ORB_SLAM3/include/ImuTypes.h`, `ORB_SLAM3/src/G2oTypes.cc`
- `covariance_enc` and `Jencg` are now present, but the edge still falls back to a hard-coded `1e4 * I` wheel block when covariance degenerates.
- Risk: parameter tuning changes behavior, but the underlying information scaling may still be structurally off.

### P4. Wheel constraints still conflict with loop behavior
- Files: `ORB_SLAM3/src/G2oTypes.cc`, `ORB_SLAM3/src/Optimizer.cc`
- Historical data shows cases where wheel changes loop candidate triggering and `BAD LOOP` rejection patterns.
- Risk: once wheel reaches graph optimization, it may change not only scale but also loop-closure eligibility.

## Experiment 1: Historical Baseline `wheel_off` vs `wheel_on`

Data source:
- `ORB_SLAM3_ROS2/experiments/ab_batch_0415`

Aggregate:

| Metric | wheel_on | wheel_off | Delta (on-off) |
|---|---:|---:|---:|
| ratio mean | 0.7903 | 0.8138 | -0.0236 |
| loop_gap mean | 0.5798 | 0.9306 | -0.3508 |
| loop_detected mean | 16.80 | 13.20 | +3.60 |
| bad_loop mean | 16.20 | 13.00 | +3.20 |

Interpretation:
- This batch confirms that wheel-related logic changed system behavior on that code/config snapshot.
- In that snapshot, `wheel_on` improved loop gap but slightly worsened ratio.
- It also triggered more loop candidates and more `BAD LOOP` rejections.
- Keep this as historical evidence, not as a direct statement about the current default path.

## Experiment 2: Historical `wheel_on` vs `wheel_on`

Data source:
- old: `ORB_SLAM3_ROS2/experiments/exp_test_0414_1808`
- current: `ORB_SLAM3_ROS2/experiments/exp_test_0415_1105`

Single-pair comparison:

| Metric | old_wheel_on | current_wheel_on | Delta (old-current) |
|---|---:|---:|---:|
| ratio | 0.6976 | 0.7077 | -0.0101 |
| loop_gap | 0.5112 | 0.7390 | -0.2278 |
| loop_detected | 20 | 0 | +20 |
| bad_loop | 19 | 0 | +19 |

Interpretation:
- This pair only shows that one wheel-enabled snapshot differed from another.
- It should not be treated as a clean answer to today's `full wheel` value question, because the runtime semantics have changed since then.
- Use it only as historical evidence that wheel behavior is highly path-dependent.

## Experiment 3: Historical Parameter A/B (`Wheel.NoiseVel`)

Data source:
- `ORB_SLAM3_ROS2/experiments/noise_0p02`
- `ORB_SLAM3_ROS2/experiments/noise_0p05`

Aggregate wheel-on metrics:

| Metric | noise=0.02 | noise=0.05 |
|---|---:|---:|
| ratio mean | 0.8496 | 0.8169 |
| loop_gap mean | 0.7001 | 0.8379 |
| loop_detected mean | 18.40 | 7.70 |
| bad_loop mean | 17.90 | 7.40 |

Against each batch's `wheel_off` baseline:

| Batch | delta ratio (on-off) | delta gap (on-off) |
|---|---:|---:|
| noise=0.02 | +0.0067 | +0.0568 |
| noise=0.05 | -0.0469 | +0.1732 |

Interpretation:
- `0.02` was better than `0.05` inside that historical wheel-on family.
- But even the better `0.02` setting did not produce a stable, clear win over its own `wheel_off` baseline.
- Treat this as evidence that tuning alone is insufficient, not as a statement about the current default config.

## Updated Judgment

Based on the historical A/B groups plus the current code-path audit:

1. Historical results show that wheel-related logic can affect scale and loop behavior.
2. Historical parameter sweeps show that tuning alone does not create a stable net win.
3. The current default runtime path is no longer the same as the older graph-on snapshots, so old wheel-on vs wheel-off data must be interpreted carefully.
4. The current default A/B only tells us about `pure IMU` vs `wheel preintegration only`.
5. The larger question, `pure IMU` vs `full wheel graph-on`, still needs an explicit dedicated comparison on the current code.

## Recommended Next Step

Do not mix the two questions below:

1. narrow question: should VIBA keep the wheel edge enabled?
2. larger question: does full wheel fusion have engineering value?

The first one is mostly answered already; the second one is still open.

Use these three modes explicitly:

1. `pure_imu_clean`
   - config: `ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final.yaml`
   - runtime: `use_wheel=false`

2. `wheel_preint_only`
   - config: `ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final.yaml`
   - runtime: `use_wheel=true`
   - note: `Wheel.UseVIBAEdge: 0`

3. `full_wheel_graph`
   - config: `ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_fullwheel.yaml`
   - runtime: `use_wheel=true`
   - note: `Wheel.UseVIBAEdge: 1`

Compare them primarily by:

- `ratio`
- `loop_gap`
- `scale_after`
- `dbg_norm`
- `dba_norm`
- reset / initialization failure count

Treat `loop_detected` and `bad_loop` as secondary side-effect signals.
