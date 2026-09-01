# Ground-station responsiveness

Why the GUI went laggy during DIRECT flight, what the cause turned out to be, and which
"obvious" fixes are already known not to work. Read this before optimising anything in
`_append_position_plot`.

## The short version

**Root cause: `setXRange`.** The live plots scrolled the *view* every tick to follow the
newest sample. Profiling the GUI thread under real DIRECT load (2026-08-04) put
`setXRange` at **49% of all GUI-thread work** — ~3 ms a call, twice per redraw — because
moving the view re-runs axis layout, the SI-prefix check and a `setHtml` label re-render
every single tick.

**Fix: plot against relative time on a fixed axis.** The x-axis is now pinned once at
`[-10, 0]` seconds and the *data* scrolls instead of the view. `setXRange` is gone from the
per-tick path entirely.

| with the plots visible | before | after |
|---|---|---|
| frames >50 ms | 17-22% | **0.0-1.7%** |
| p95 frame interval | 65-69 ms | **34.7 ms** (healthy = 33.3) |
| worst click latency | 43 ms | **9.7 ms** |

The plots stay on screen. Earlier revisions of this document told the operator to switch
away from the `Flight Log` sub-tab during DIRECT flight — **that is no longer necessary.**

## Metric

`update_gui_data()` is driven by a 30 Hz timer, so the interval between consecutive calls
should be 33 ms. The same thread paints the widgets, so inflation of that interval *is* the
lag the operator feels.

Use **frames >50 ms** — the share of GUI frames that took more than 1.5x their budget.
Median is useless (it sits at ~33 ms in every configuration, healthy or not). p95 is worse
than useless when redraws are decimated: the slowest 5% of frames are then *always* redraw
frames, so p95 measures the size of a hitch and is blind to how often one happens. That is
exactly why redraw decimation looked like it did nothing.

Load in all measurements is taken from the 2026-08-03 flight bags, not invented:

| topic | rate | GUI callbacks/s |
|---|---|---|
| `fmu/out/vehicle_attitude` | 200 Hz | 200 |
| `direct_actuation/motors_debug` | 250 Hz | 250 |
| `attitude_setpoint_debug` | 100 Hz | 100 |
| `state_estimator/local_position/odom` | 60 Hz | 120 (**was subscribed twice**) |
| `fmu/out/vehicle_status_v1` | 2 Hz | 2 |
| | | **672 in DIRECT**, ~420 in SAFETY |

## Profile that found it

25 s of DIRECT load, plots visible, cumulative time inside `update_gui_data`:

| | cum | share |
|---|---|---|
| `update_gui_data` total | 3.534 s | 100% |
| └ `_append_position_plot` | 2.625 s | 74% |
| &nbsp;&nbsp;└ **`setXRange` chain** | **1.735 s** | **49%** |
| &nbsp;&nbsp;└ `setData` (10 curves) | 0.833 s | 24% |

Note `paintEvent` of the rotor-speed widget does not appear at all.

## Things that were tried and do NOT work

Each was measured against the dropped-frame rate. None of them changed it. Do not retry
these.

| attempted fix | result |
|---|---|
| Disabling the rotor-speed / throttle-pie widget | −0.8 ms p95 in DIRECT, −1.3 in SAFETY, i.e. nothing |
| `setDownsampling(auto=True, mode='peak')` + `setClipToView(True)` | **worse** (p95 49.4 → 58.5); at 300 points the downsample computation exceeds the draw cost |
| `pg.setConfigOptions(antialias=False)` | no change |
| Batching with `setUpdatesEnabled(False/True)` | no change |
| Disabling Y auto-range, fixed `setYRange` | no change |
| Fewer points per curve (300 → 150 → 60) | no change |
| Redraw decimation 30 → 15 → 10 → 6 → 3 Hz | no change in dropped frames |
| Cutting `motors_debug` 250 → 25 Hz | no clear effect |
| Cutting attitude 200 → 50 Hz, setpoints 100 → 25 Hz | no clear effect |
| **Zero ROS traffic**, plots live | still 18-25% — message load was never the cause |

## Hypotheses that were disproved by measurement

Worth recording so nobody re-derives them:

- **Qt event-queue backlog.** `timer_callback` emits `update_data` from the ROS thread into
  a queued connection, so a slow GUI thread could in principle build an unbounded backlog
  with clicks stuck behind it. Measured: backlog ~0, click latency sub-millisecond. Not it.
- **Autopilot service latency.** The node spins with `rclcpp::spin()` — single-threaded, no
  callback groups — and its 250 Hz inner-loop timer early-returns in SAFETY but runs the
  full attitude+rate+allocation cascade in DIRECT, so service replies might queue behind it.
  Measured against the real node at flight rates: `list_controllers` round-trip **1.37 ms in
  SAFETY vs 1.31 ms in DIRECT**. Not it.
- **`publish_coordinates` on the Qt thread.** An ordered A/B suggested 5.26x, a rerun of the
  same test gave 1.08x — it was measuring drift. Still worth tidying (see below), but it was
  not the cause.

## What was actually changed

1. **Fixed relative-time x-axis** on the position and body-angle plots — the actual fix.
   `setXRange` is called once at setup; `_append_position_plot` now emits `t_i - t_now` so
   the newest sample sits at x=0. Also `enableAutoSIPrefix(False)` on the bottom axes, and
   the unit moved into the label text, so pyqtgraph never re-renders the label.

2. **Visibility gate** (`_append_position_plot`, `_append_step_response`). Curve updates are
   skipped while the plots are off-screen. This was the original workaround; it is retained
   as defence in depth and still measures 0.0%. The buffers keep filling, so switching back
   shows complete history. The step-response plot flushes once when it becomes visible, so a
   step that completed on a hidden tab is not left as a partial trace.

3. **Merged the duplicate odometry subscription.** `state_estimator/local_position/odom` was
   subscribed twice — once for pose, once for twist — deserialising every message twice for
   data that arrives in one message. 672 → 613 callbacks/s. Free, but not a lag fix.

4. Redraw decimation (`PLOT_REDRAW_EVERY = 3`) is **kept but is not the fix** — it cuts plot
   CPU ~3x, and 10 Hz is adequate for monitoring. Do not cite it as the fix.

## Still open

- `send_coordinates` calls `publish_coordinates` on the **Qt thread**, where it touches the
  clock, the publisher and the logger while the rclpy executor spins. The controller-service
  buttons were moved to a queue drained on the ROS thread; this one never was. Measured cost
  is inconsistent, so this is a correctness/consistency cleanup, not a performance fix.
- `log_message` appends to an **unbounded** `QListWidget` and calls `scrollToBottom()` every
  time. All 23 call sites are edge-triggered so it does not spam, but nothing caps growth
  over a long session.
