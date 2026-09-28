# AGENTS.md

## Project overview

This repository contains a PyQt5 ground-control GUI for ROS 2 and PX4-based drone experiments. There are two entry points: `src/GroundControl.py` launches a multi-drone station for UAV IDs `0`, `1`, and `2`; `src/single_drone_ground_control.py` launches a single-UAV station (`uav_0`) built around a dedicated layout for single-drone flight experiments.

The application is not packaged as an installable Python or ROS package. Imports and configuration paths assume the command is run from the repository root; running a script under `src/` puts `src` on the import path by itself.

## Repository map

- `src/GroundControl.py`: multi-drone application entry point; initializes `rclpy`, creates the Qt application, and starts `ros_multi_drone_control.MultiDroneRosThread`.
- `src/single_drone_ground_control.py`: single-drone application entry point; initializes `rclpy`, creates the Qt application from `GUI/single_drone_flight.py`, and starts `ros_single_drone_control.SingleDroneRosThread`.
- `src/ROS_Node/ros_multi_drone_control.py`: active ROS 2 multi-drone node, topics, publishers, Qt signal/slot connections, and GUI updates. Used by `GroundControl.py`.
- `src/ROS_Node/ros_single_drone_control.py`: active ROS 2 single-UAV (`uav_0`) node, topics, publishers, Qt signal/slot connections, and GUI updates. Used only by `single_drone_ground_control.py`.
- `src/ROS_Node/ros_water_sample_control.py`: legacy ROS 1 (`rospy`) water-sampling implementation; currently not exported or started.
- `src/ROS_Node/ros_common.py`: small data-holder classes shared by the controllers.
- `src/Common/common.py`: mutex-protected telemetry/state shared between ROS callbacks and Qt updates; includes frame and quaternion conversions.
- `src/Common/llm_client.py`: Qt-only client for the LLM proxy (health check, streamed chat), used by the single-drone VLM tab. No rclpy.
- `src/GUI/GUI_Sampler.ui`: Qt Designer source of truth for the multi-drone GUI.
- `src/GUI/GUI_SamplerdotUI.py`: generated PyQt5 code imported by `GroundControl.py`.
- `src/GUI/single_drone_flight.ui`: Qt Designer source of truth for the single-drone GUI (an "Autonomous Flight" telemetry tab plus an "Initialization" tab with arm/disarm/takeoff/land/mode controls).
- `src/GUI/single_drone_flight.py`: generated PyQt5 code imported by `single_drone_ground_control.py`.
- `src/ROS_Node/geofence.json` and `slung_load.json`: runtime configuration.
- `scripts/convert_ui.sh`: converts a `.ui` file to Python with `pyuic5`.
- `docs/prerequisites.md`: ROS 2 Humble, Python, message-package, and sibling-workspace requirements.
- `GUI_SamplerdotUI.py`: legacy generated file at the repository root; the active import uses the copy under `src/GUI`.

## Environment and dependencies

Use a ROS 2 environment that provides the workspace's message packages. Important runtime dependencies include:

- Python 3, PyQt5 (including QtNetwork, which ships with it), and NumPy
- ROS 2 Python (`rclpy`)
- `px4_msgs`
- `fsc_autopilot_ros2_msgs`
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, and `visualization_msgs`
- `pyqtgraph` (optional): powers the embedded real-time plots in the single-drone GUI (`display_x_y_z` and `display_body_angle` — see "Single-drone telemetry plots"). Import is guarded — the GUI runs without it, printing `[PLOT] pyqtgraph not found` and leaving those plots blank.
- `spd-say` from speech-dispatcher, and `pacat` (or `aplay`) (all optional): the spoken OptiTrack/WiFi announcements and the siren in front of a loss, in the single-drone GUI (see "Audio alarms"). Without them the GUI prints `[AUDIO] ... not found`, and the indicators still work silently.

See `docs/prerequisites.md` for the expected ROS 2 workspace layout and upstream packages. In particular, `px4_msgs` and `fsc_autopilot_ros2_msgs` must be available from the sourced workspace.

Source the ROS installation and the containing workspace before running the GUI. From this repository root, launch the multi-drone station with:

```bash
python3 src/GroundControl.py
```

or the single-drone station with:

```bash
python3 src/single_drone_ground_control.py
```

Do not prefix `PYTHONPATH=src`: it *replaces* the sourced ROS entries on `PYTHONPATH` and
the GUI dies with `No module named 'rclpy'` (checked 2026-09-26). Python already puts the
script's directory, `src/`, first on the import path. If you must set it, use
`PYTHONPATH=src:$PYTHONPATH`. Close the window to quit; Ctrl+C in the terminal does not
stop the Qt event loop.

Run from the repository root because the controllers currently open JSON configuration using paths such as `src/ROS_Node/geofence.json`.

## Single-drone telemetry plots

Two pyqtgraph plots live on the "Flight Log" tab, each drawn into a bare `QWidget`
container declared in `single_drone_flight.ui` and filled in at runtime by
`ros_single_drone_control.py`. Both share one time axis and one history window
(`POSITION_PLOT_HISTORY_S`), and both are fed from the single `_append_position_plot()`
tick, so their sample sets stay aligned.

| Container | Setup method | Curves |
| --- | --- | --- |
| `display_x_y_z` | `_setup_position_plot_impl()` | X/Y/Z position (m) solid, X/Y/Z command dashed |
| `display_body_angle` | `_setup_body_angle_plot()` | Roll/Pitch/Yaw (deg) solid, Yaw command dashed |

Conventions that are easy to break:

- **The x-axis is fixed at `[-10, 0]` s and must stay that way.** Curves are fed
  `t_i - t_now`, so the newest sample sits at x=0 and the *data* scrolls, not the view.
  Do **not** "fix" this back to absolute time plus a per-tick `setXRange` — profiling
  under real DIRECT-mode load (2026-08-04) put that single call at **49% of all
  GUI-thread work**, because moving the view re-runs axis layout, the SI-prefix check and
  a `setHtml` label re-render every tick. It was the cause of the ground station becoming
  unusable during flight. Both bottom axes therefore also set `enableAutoSIPrefix(False)`
  and carry their unit in the label text, for the same reason as the degrees rule below.
- **Curve updates are skipped while the plots are off-screen** (an `isVisible()` gate),
  with the deques still filling so no history is lost. Retained as defence in depth after
  the axis fix. `_append_step_response()` additionally flushes once when its plot becomes
  visible, so a step recorded on a hidden tab is not left as a partial trace.
- Before optimising anything here, read [docs/gui_responsiveness.md](docs/gui_responsiveness.md):
  antialiasing, batched updates, fixed Y range, `setDownsampling`/`setClipToView`, point
  count (300 → 60) and redraw decimation (30 → 3 Hz) were each measured and **none** of
  them help; `setDownsampling` made it worse. Message load is not the cause either.
- **Position and attitude are deliberately on separate plots.** Yaw previously shared
  `display_x_y_z` on a second right-hand `ViewBox` with its own axis, which needed a
  `sigResized` handler to keep the two viewboxes aligned. That is all gone — the
  position plot now has a single left axis in metres. Do not reintroduce a second
  ViewBox without also restoring the resize syncing.
- **Only yaw has a commanded counterpart.** Roll and pitch are outputs of the position
  controller, not operator commands, so they have no dashed line. Their setpoints would
  have to come from a new subscription (e.g. `attitude_setpoint_debug`), not from
  existing GUI state.
- **Angles are already degrees** when they reach the plot: `common.py`'s
  `quat_to_euler()` converts before storing. Roll and pitch arrive as ±180; **yaw
  arrives wrapped to [0, 360)** and is re-normalised to ±180 in `_append_position_plot()`
  to match `_last_yaw_cmd`, which `send_coordinates()` already stores as ±180.
- **Label degrees as text (`'Body angle (deg)'`), never `units='deg'`.** pyqtgraph
  SI-prefixes unit strings and will render "mdeg"/"kdeg" as the range changes.
- The body-angle plot's Y range is **pinned to ±180** rather than autoscaled; autoscale
  turns a level hover into apparent violent oscillation by zooming into millidegree
  noise.
- `_plot_err_x/y/z` are still appended and trimmed but no longer plotted anywhere —
  the position-error plot they fed was replaced by the body-angle plot. Harmless (they
  are still trimmed, so no unbounded growth), but do not assume they are displayed.

## Single-drone step-response logging

The "Step Response" tab (`single_drone_flight.ui`) plots position response to a single
commanded step, separate from the continuous telemetry plots on "Flight Log". Recording
does not start on every `send_coordinates()` call — it must be armed explicitly first
(added 2026-07-31):

- `buttom_enable_log` toggles `_pref_armed` via `_toggle_enable_log()`. Its label tracks
  state: `"Enable"` when disarmed, `"Waiting..."` when armed.
- `send_coordinates()` only calls `_start_step_response(x, y, z)` if `_pref_armed` is
  `True` at send time; doing so clears the flag and resets the button label, so arming
  is a single-shot latch that captures exactly one subsequent send, not a persistent
  recording mode.
- `buttom_pref` ("Reset") clears the currently plotted window at any time and is
  independent of arming.
- `_pref_waiting`/`_pref_recording` track whether a capture is in progress versus
  finished/idle; they do not gate whether a capture starts — `_pref_armed` does.

Keep the arm state single-shot when extending this workflow, so an accidental repeated
send doesn't silently overwrite a capture the user meant to keep.

## Single-drone OptiTrack indicator

`optitrack_status` (a `QLabel` on the Autonomous Flight tab) has a green background with
"OptiTrack normal" while raw OptiTrack frames arrive. After `OPTITRACK_TIMEOUT_S`
(0.4 s) without a frame it turns red with "No OptiTrack". Each transition is announced
and written to the flight log. Recovery after a loss is announced as "OptiTrack
regained"; present at launch is just "OptiTrack normal". Every "No OptiTrack" gets a
siren before the words. See "Audio alarms" (added 2026-09-25; background colours and
siren since 2026-09-26).

- **It watches `/vrpn_mocap/uav_0/pose` (`OPTITRACK_TOPIC`), not `/uav_0/mocap`.**
  During a VRPN dropout, `fsc_optitrack_processor_ros2` keeps republishing the held
  pose on `/uav_0/mocap` at its timer rate. The estimator's `/uav_0/mocap_status`
  watchdog watches `/uav_0/mocap`, so it too keeps saying `mocap_normal` with
  OptiTrack gone. The Orin's `fsc_system_monitor` also watches `/uav_0/mocap`, so its
  `mocap.status` would read "degraded" (the ~60 Hz held-pose rate) rather than "lost".
  Do not "simplify" the indicator onto any of those. The topic name comes from the
  Motive rigid-body name (`uav_0`).
- **Timeout choice:** in the 2026-09-24 flight bag, VRPN ran at ~120 Hz and its largest
  gap was 23 ms. 0.4 s therefore means ~48 missed frames and will not trip on WiFi
  jitter. Tested against that bag: red 0.41 s after the last frame (timeout plus one
  GUI tick).
- **The subscription is `raw=True`** because only arrival time is used. This avoids
  deserialising ~120 messages/s on the ROS thread.
- **Startup:** for `OPTITRACK_STARTUP_GRACE_S` (2 s) the GUI does not declare a loss,
  which gives DDS discovery time. The first verdict is then announced once either
  way, so launching the station tells you where OptiTrack stands.
- **Updates are edge-triggered.** Announcements only happen on a transition, and all
  three status labels go through `_set_status_label()`. It calls
  `setText`/`setStyleSheet` only when the value changes, because `setStyleSheet`
  re-polishes the widget and should not run at 30 Hz.
- **Every "No OptiTrack" gets the siren, the launch verdict included.** At the desk
  with no mocap it therefore sounds on every launch. It was briefly exempted to avoid
  alarm fatigue, but the operator wants it (2026-09-26). If mocap is running but
  discovery takes longer than the 2 s grace, the siren is cut short by
  "OptiTrack regained".

## Single-drone Orin status (`cpu_status`, `wifi_status`)

Both labels come from one topic, published by `fsc_system_monitor` on Orin0 (added
2026-09-26):

| Topic | Type | Rate | GUI reader |
| --- | --- | --- | --- |
| `/uav_0/system_monitor/status` | `std_msgs/String` (JSON, ~600 B) | 1 Hz, published reliable | best-effort, depth 1 |

The JSON has `stamp` (Orin clock, display only), `wifi.*`, `cpu.*`, `mocap.*` and
`odom.*`, and any field may be `null`. The GUI uses `cpu.load_pct` (six values:
CPUs 0-3 housekeeping, 4 the uXRCE-DDS Agent, 5 the control node) and the `wifi` block.
The formatting and thresholds are the module-level functions `cpu_summary()` and
`wifi_summary()`, so they can be checked without Qt or ROS.

| `wifi_status` | When |
| --- | --- |
| green "WiFi good" | none of the conditions below |
| yellow "WiFi: <reason>" | `signal_dbm` < -75, `ping_ms` null or > 50, `qdisc_backlog_max_pkts` > 0 in 3 consecutive reports, or `udp_txq_max_bytes` > 100 KB |
| red "No WiFi" | no report for `SYSTEM_STATUS_STALE_S` (3 s), or `wifi.connected` is false |

- **"No WiFi" is mostly judged from silence.** The Orin is WiFi-only, so from the ground
  station a dead link and a dead monitor look the same. The second line of the label
  says which one it saw. `connected: false` can only arrive if some other route
  exists.
- **Best-effort reader on purpose.** A reliable reader would make the Orin retransmit
  stale reports over a link that is already stalling.
- **Two lines at 9 pt** (`TWO_LINE_FONT`). Checked to fit the 231x41 and 391x41
  labels with the longest texts, e.g. "no status from Orin for 125 s" and six loads
  at 100%.
- **Only entering and leaving "No WiFi" is announced and logged**: siren and
  "No WiFi", then "WiFi regained" without a siren. Good <-> fair can flip every second
  while the signal sits near a threshold. At launch the verdict waits
  `SYSTEM_STATUS_STALE_S` (3 s) for a first report; if none has come, launching with
  the Orin off is announced like any other loss (siren and "No WiFi"). A good first
  verdict is not announced.
- `mocap`/`odom` from this report are not shown. See the OptiTrack section for why
  `mocap.status` is not used for `optitrack_status`.

## Audio alarms (`AudioAnnouncer`)

Announcements play one at a time: a siren (optional), then speech. Every step is a
`QProcess`, so nothing blocks the GUI thread.

- **The siren is generated in memory** (`_siren_pcm()`: three 700-1400 Hz sweeps,
  1.2 s). It is piped as raw PCM to `pacat`, or `aplay` if `pacat` is missing, so no
  sound file lives in the repo. Speech is `spd-say -w`, which exits once the text has
  been spoken, and that is what sequences the queue.
- **Per-source supersession.** Each announcement has a source (`'optitrack'`,
  `'wifi'`). A newer one from the same source drops that source's queued item and
  kills its playing one, and speech is cancelled with `spd-say -C`, because killing
  the client does not reliably stop speech-dispatcher. Without this, a loss followed
  by a quick recovery would play "OptiTrack regained" during the siren and then the
  stale "No OptiTrack" after it. Announcements from different sources queue instead,
  because OptiTrack and WiFi tend to drop together and neither alarm should clip the
  other.
- **Watchdog.** A step that runs past `STEP_TIMEOUT_MS` (10 s) is killed and the
  announcement continues, so a hung siren player still lets the words through.
- Tested 2026-09-26 with stand-in players (ordering, supersession, both sources at
  once, hung siren, hung speech) and silently through the real `pacat`/`spd-say`
  (siren 1.2 s, then speech, no leftover processes).

## Single-drone LLM status assistant (VLM tab)

The operator types a question in `LLM_input`, and a language model answers in
`LLM_chatlog` from what this GUI has received (added 2026-09-26). Status answers are
read-only. The one exception is the camera search (see "VLM camera search" below): it
can *propose* small moves, and only the operator's click sends one.

| Widget | Role |
| --- | --- |
| `buttom_connect_VLM` | Connect / Disconnect. Runs a health check, preloads the model, and polls health every 15 s while connected |
| `LLM_input` | the question. Enter sends; Shift+Enter adds a line |
| `LLM_send` | Send; becomes Stop while an answer streams |
| `LLM_chatlog` | the conversation, bounded to `LLM_CHATLOG_MAX_BLOCKS` |
| `Api_status` | not connected / connecting / loading model / online / unreachable. **Optional**: if the `.ui` has no such label the station still starts and prints a warning |

**Server.** The LLM host's proxy at `LLM_BASE_URL` (a NetBird address), model
`LLM_MODEL` (`qwen2.5vl:7b`). It exposes `GET /api/health` and `POST /api/chat`. Ollama
itself stays on loopback on the LLM host. The proxy has **no authentication**, so any
NetBird peer the access policy allows can use it. Proxy behaviour observed 2026-09-26:

- it forwards only `model` and `messages`, and always streams Ollama-format NDJSON
  (one JSON object per line). It ignores `stream` and `options`, so temperature and
  `num_predict` have no effect.
- it rejects an empty `messages` list, so the preload on connect is a tiny real
  request.
- errors come back as `{"error": "..."}`, e.g. with HTTP 400.
- the model unloads after 5 min idle; the next answer then waits 5-10 s to reload.

Rules for this path:

- **Read-only status answers.** The model gets a telemetry snapshot and answers in
  text; the proxy's `/api/parse` is not used. The only paths from here to the vehicle
  are the camera search's snapshot request, which is harmless, and its proposed moves.
  Moves are gated by the operator's click and by the GUI's own checks: geofence, pose,
  armed, OFFBOARD, yaw alignment and OptiTrack. Any future LLM command path must keep
  that shape. `/api/parse` returns only what the model said, and the geofence and pose
  checks live in `nl_commander`, not on this path.
- **Threading.** `QNetworkAccessManager` runs on the GUI thread
  (`src/Common/llm_client.py`): it is asynchronous, never blocks, and never touches
  rclpy. The snapshot reads `CommonData` with `tryLock(50)`: a bounded wait once per
  question, not per tick.
- **Snapshot (`_llm_telemetry_snapshot()`).** Built fresh for each question and put in
  that question's user message. It is not kept in the history, which holds only plain
  question/answer text (the last `LLM_HISTORY_TURNS` pairs). Every source carries
  `age_s` from `CommonData.last_update`, and a source never received says "no data
  received" instead of the zeros the data holders start with. It covers:
  - vehicle and PX4 status;
  - position and velocity (local mocap frame, z up);
  - attitude (yaw ±180);
  - the last position command sent, and position error;
  - battery (PX4 `remaining` 0..1, shown as %);
  - motor commands, when fresh;
  - OptiTrack;
  - WiFi, CPU and onboard feed status from `system_monitor`;
  - camera detections;
  - the last 8 flight-log lines.

  That is roughly 600 tokens.
- **Streaming.** Text reaches the widget at most 10x a second (`LLM_FLUSH_MS`). A
  finished answer is re-rendered from markdown, because the 7B model uses markdown for
  longer answers despite the prompt. The answer is located as the last `len(answer)`
  characters of the log, and replaced only if they match. A saved `QTextCursor` cannot
  anchor it, because Qt moves the cursor when a newline is inserted at its position.
  Status lines are not written to the log while an answer streams; they would land
  inside it.
- **Timeouts.** Health checks time out after 5 s. Chat has a 45 s inactivity timeout
  (Qt's `transferTimeout`, which resets whenever data arrives).
- **Tested 2026-09-26.** Against the live proxy, with a replayed flight:
  - answers took 0.7-1.5 s;
  - an arm request was refused;
  - Stop, Shift+Enter and Disconnect all worked;
  - with no telemetry, the model said the armed state was unknown.

  The client's edge cases (HTTP 400 body, a cut stream, a stall timeout,
  abort-then-ask) were tested against a local fake server.

## VLM camera search ("is there a bottle in view?")

Added 2026-09-27. The operator asks in the chat. Explicit camera requests ("is there a
bottle in view?", "look for a person") are recognised in code (`parse_look_request`).
The chat model's `{"action": "look", "object": ...}` reply remains a fallback for
unusual phrasing. The GUI then:

1. **Snapshot.** It asks the Orin for its latest frame.
2. **VLM verdict.** The VLM judges whether the object is in view.
3. **Detector match.** The detector's result for that same frame is matched by label.
4. **Found:** it stores and reports a room position.
5. **Not found:** if **LLM Control** is on, it proposes up to `LOOK_MAX_MOVES` (5)
   relocation moves. **Each move needs a Commit Action click.**

| Widget | Role |
| --- | --- |
| `buttom_LLM_commit_2` ("LLM Control") | checkable gate for moves; **off at every launch**; logged on change. Turning it off ends a search with a pending or in-progress move |
| `buttom_LLM_commit` ("Commit Action") | enabled only while a move is pending; its text names the move, e.g. "Commit: turn left 30°"; a click sends exactly that move |
| `LLM_control_status` | "LLM control on" (green) / "LLM control off" |

All three are optional, like `Api_status`: without them the search reports but never
moves. `buttom_LLM_commit_2` is Designer's copy name; renaming it in Designer means
updating the `getattr` in `_setup_llm`.

| Interface | Type | Notes |
| --- | --- | --- |
| `/uav_0/snapshot/request` | `std_msgs/String` JSON `{"id", "quality"}` | GUI → Orin, reliable |
| `/uav_0/snapshot/response` | `std_msgs/String` JSON, ~85 kB | Orin → GUI, reliable. One message holds `jpeg_b64` (640x480, quality 95), that frame's `detections` (label, confidence, bbox, `position_camera`, `position_body`), `detector_classes`, `body_T_color`, `frame_age_ms` |
| `/uav_0/mocap` | `fsc_autopilot_ros2_msgs/Mocap` | subscribed `raw=True` and decoded only when a snapshot is requested |

The responder lives in the Orin's `realsense_camera_object_localization` (branch
`gs-snapshot`, `ros_publisher.py`), inside the detector process, which owns the
camera. It answers from a background executor thread; the camera loop never waits on
it. Measured live: ~160 ms round trip, frames 15-40 ms old.

- **Room position = R(q) · `position_body` + t**, using the `/uav_0/mocap` pose sampled
  on the ROS thread when the request left. Use `/uav_0/mocap`, not the estimator's
  odom: the camera extrinsic was calibrated against `/uav_0/mocap`, and odom lags it by
  up to ~10 cm / 5 deg in flight. `body_to_world()` reproduces the Orin calibration's
  own AprilTag check to 2.3 cm RMS (8 views).
- **`body_T_color` is the `extrinsics.yaml` matrix as-is.** The 2026-09-25 calibration
  was solved from points in the *color* optical frame, which is the frame the detector
  reports in. Do not route it through `load_drone_to_color()`, which is meant for CAD
  values anchored on the depth module; that would add the depth-to-color offset
  (~15 mm) a second time.
- **The VLM judges only the image.** Asked in one JSON reply for visibility, the index
  of the matching detector box and a move, qwen2.5vl:7b answered from the detector
  list and called a plainly visible bottle "not visible". The fix was a describe-first
  reply plus `{"visible", "move"}`, with the detector matched by label in code
  (`_match_detection`). Keep it that way.
- **Moves.** Only the fixed set in `LOOK_MOVES` is allowed: yaw ±30°, ±0.3 m ahead or
  to the side in the heading frame, ±0.2 m vertical.
  - When the object isn't in view and the VLM suggests nothing, the GUI proposes a
    scanning turn (yaw left).
  - A move is only offered if the drone is armed, in OFFBOARD, yaw-aligned and has
    OptiTrack, and the target is inside the geofence.
  - These are all re-checked when Commit Action is clicked. If the drone has shifted
    more than 0.2 m or 10 deg since the proposal, nothing is sent.
  - Moves are sent through `_issue_position_command()`, the same path as manual
    commands.
- **No dialog.** The operator answers on the tab itself, so every other control,
  back-to-baseline included, stays usable while a move is pending. Stop (`LLM_send`),
  LLM Control off and Disconnect end the search at any stage.
- **Routing in code, not by the model.** With an earlier result in its snapshot,
  qwen2.5 answered a repeat "look for a person" from memory 4 times out of 4, and a
  stronger prompt fixed only 1 in 4. `parse_look_request` catches the explicit
  phrasings, skips the routing call (about 1 s faster), and leaves past-tense
  questions ("did you find ...?") and telemetry words ("can you see the WiFi status?")
  to the model. Label matching knows `people` -> `person` (`object_matches_label`).
- **History.** The chat history keeps the model's `{"action": "look"}` reply. Results
  reach it through the snapshot (`recent_camera_checks`, `found_objects`). Storing the
  result text as the model's reply broke "did you find a person earlier?".
- **Tested 2026-09-27.**
  - Live against the Orin camera and the real VLM: a bottle was found in 1.7 s; the
    chair was reported as not detectable; the person scan was refused because the
    drone was disarmed. Command publishing was stubbed out.
  - On an isolated domain with a replayed flight, through the real widgets:
    - a 5-move scan with Commit Action clicks;
    - a room position (bottle on the floor, z ≈ 0.05 m);
    - LLM Control off with a move pending;
    - a search with control off, which proposed no move;
    - Stop during analysis.
    - 6 commands reached the recorder, as expected.
  - Not yet flown.

## Single-drone camera view

`cam_vision_0` on the "VLM" tab shows the onboard RealSense color stream with the
YOLO detector's boxes drawn over it (`CameraDetectionView`, added 2026-09-25). The chat
widgets on the same tab are described in "Single-drone LLM status assistant".

| Topic | Type | Published QoS | GUI reader |
| --- | --- | --- | --- |
| `/uav_0/color/compressed` | `sensor_msgs/CompressedImage` (JPEG, 640x480) | best-effort | best-effort, depth 1 |
| `/uav_0/detections` | `std_msgs/String` (JSON) | reliable | best-effort, depth 5 |

The publisher is the `vision_localization` node, i.e. `main.py --ros` in
`~/dev_ws/src/realsense_camera_object_localization` on Orin0. It is not in this workspace.

- **The namespace comes from that script's `--ros-ns` flag**, not from this repo. It
  was `/vision` for the 2026-09-24 bench bag and `/uav_0` from 2026-09-25. If it
  changes, update the `VISION_*_TOPIC` constants in `ros_single_drone_control.py`.
  A missing `--ros` flag has the same symptom as a wrong namespace: no topics, only
  the MJPEG stream on :8080.
- **Rates depend on the publisher's flags.** Live on 2026-09-25, images and detections
  both ran at ~24 Hz. The bench bag had images at 10 Hz (`--ros-image-hz 10`) and
  detections at ~30 Hz.
- **Desk test without the drone:** replay `~/ros2bag/vision_bench_20260924_181035` on
  an unused `ROS_DOMAIN_ID` with `ROS_LOCALHOST_ONLY=1`. The bag uses the old
  namespace, so remap it:
  `--remap /vision/color/compressed:=/uav_0/color/compressed /vision/detections:=/uav_0/detections`.
- **Frames and boxes are paired by header stamp, not by arrival.** The JSON `stamp`
  is the stamp of the color frame the boxes came from. Images can be published at a
  lower rate than detections (about every third frame in the bench bag), and the two
  topics arrive independently. Drawing the latest detections on the latest image
  would therefore put boxes from a different moment on the frame. `CommonData.match_vision_detections()` looks for the exact stamp first,
  then the nearest one within 60 ms, and otherwise returns no boxes.
- **`bbox` is in pixels of the published image.** The view scales boxes with the same
  transform as the letterboxed frame. If the publisher starts downscaling the JPEG
  without scaling `bbox` to match, the boxes will drift.
- **Frames are stored undecoded and decoded only while the view is visible.** The
  same `isVisible()` gate is used for the plots. `QImage.fromData` takes ~1.2 ms per
  frame and holds the GIL. When visible, the view costs ~0.04 ms per tick plus a
  ~1.3 ms repaint per new frame (measured against the bag, 2026-09-25). At the live
  ~24 Hz image rate, that is roughly 6% of the GUI thread while the tab is showing.
- **Staleness is judged by receive time (`time.monotonic()`), not header stamps**,
  because the Jetson's clock is not synchronised with the ground station's. After
  `VISION_STALE_S`, a frozen frame is dimmed and the status line turns red. "Detector
  silent" is reported separately from "0 object(s)", so that "no detector" and
  "nothing detected" can be told apart.

## Editing rules

- Treat `src/GUI/GUI_Sampler.ui` and `src/GUI/single_drone_flight.ui` as the source of truth for GUI layout changes, for the multi-drone and single-drone stations respectively. Do not manually edit the corresponding generated `*.py` files; they contain a generated-file warning and will be overwritten.
- Regenerate the active UI after editing it (run from the repository root; the script reads from and writes to `src/GUI`):

  ```bash
  ./scripts/convert_ui.sh GUI_Sampler.ui GUI_SamplerdotUI.py
  ./scripts/convert_ui.sh single_drone_flight.ui
  ```

- Review the generated diff after regeneration to ensure the `.ui` and Python output remain synchronized.
- Use `scripts/convert_ui.sh` rather than calling `pyuic5` by hand. The script passes `-x`, which emits the trailing `if __name__ == "__main__":` preview block; a bare `pyuic5` omits it, so a hand-run regeneration silently deletes that block and shows up as unrelated noise in the diff.
- Keep Qt widget `objectName` values stable unless every corresponding reference in the ROS controller is updated. Renaming a container in Qt Designer does **not** fail loudly — the generated `*.py` picks up the new name while `ros_single_drone_control.py` still asks for the old one, and the GUI dies with an `AttributeError` only when that plot is first built. After any rename, grep the controller for the old name. (Precedent: `display_tracking_error` -> `display_body_angle` and `display_x_y_z_yaw` -> `display_x_y_z`, 2026-07-30.)
- Preserve the ROS/Qt threading boundary. ROS executors run in `QThread`; GUI widgets must be updated through Qt signals/slots on the GUI side.
- **The boundary is two-way: nothing on the Qt thread may touch rclpy.** Button handlers run on the Qt thread while the executor spins, so calling a publisher, a service client, `get_clock()` or `get_logger()` from one contends for the rclpy locks. Push the request onto `_pending_requests` (guarded by `_request_lock`) through a `queue_*` helper and let `_drain_requests()` — called from `timer_callback()`, i.e. the ROS thread — issue it on the next tick (≤33 ms). Existing helpers: `queue_controller_list()`, `queue_controller_activation()`, `queue_coordinates()`. The last was added 2026-08-04 after `send_coordinates()` was found publishing inline; the first two came from the switch-controller button stalling ~1 s. Cache service *readiness* too (`_refresh_services_ready()`, throttled to 2 Hz) — graph queries are slow and lock-contended.
- Keep the flight log bounded. `log_message()` trims `list_cmd_log` to `LOG_MAX_LINES`; an unbounded `QListWidget` makes every `scrollToBottom()` progressively slower over a long session.
- Keep publishers, subscriptions, clients, timers, executors, and threads referenced for as long as they are needed. Shutdown must not leave an executor or worker thread running.
- Access shared `CommonData` state under its `QMutex`. Keep lock sections short, copy values locally, always unlock on every acquired path, and preserve the existing non-blocking `tryLock()` behavior unless deliberately changing the concurrency model.
- For each new UAV-specific topic, derive the namespace from the drone ID (`/uav_{i}/...`) and store subscriptions/publishers so ROS objects remain alive.
- Match PX4 QoS behavior: telemetry uses best-effort/transient-local/keep-last/depth-5, while PX4 command and setpoint debug streams use best-effort/volatile/keep-last/depth-5 unless the upstream interface explicitly requires otherwise.
- Check the sourced workspace's message definitions before changing custom message fields.
- Be explicit about coordinate frames and units. PX4 data is commonly NED/FRD, while the GUI and estimator may use ENU/FLU; quaternion ordering differs across interfaces, and `PositionControllerReference.yaw` is published in degrees. Do not silently change axis order, quaternion order, yaw wrapping, thrust signs, frame IDs, or other conversion conventions.
- Avoid broad cleanup of commented legacy ROS 1 (water-sampling) code unless the task specifically includes migration or removal.
- Keep configuration JSON valid and preserve its existing schema unless all readers are updated together.

## Controller-tab design direction

The single-drone GUI's Controller tab is intended to discover and activate controllers;
it is not a fixed two-mode display:

- `avaliable_controllers` displays every controller reported as available by the running
  ROS 2 control stack. Preserve this existing widget `objectName` despite its spelling.
- `buttom_refresh_options` sends a ROS 2 service request to query the available
  controllers. The request must be asynchronous and have a finite timeout so an absent
  or unresponsive service never freezes the Qt GUI. On timeout or service failure, keep
  the previously displayed list and report the error in the command log/status UI.
- `buttom_activate_controller` requests activation of the controller currently selected
  in `avaliable_controllers`. Disable it when there is no valid selection, while a request
  is pending, or when the activation service is unavailable. Report the service result
  and confirm the active controller from ROS status feedback rather than assuming that a
  sent request succeeded.
- `buttom_back_to_baseline` is the dedicated control for switching back to the baseline
  controller. It does not depend on the table selection. Keep this return path distinct
  from `buttom_activate_controller`, and confirm the transition from ROS status feedback.
  Preserve the existing widget `objectName` despite its spelling.
- The interface (implemented 2026-07-30, `fsc_autopilot_ros2_msgs`):

  | Purpose | Interface |
  |---|---|
  | Discovery | `/uav_0/fsc_autopilot_ros2/list_controllers` (`ListControllers`) |
  | Activation | `/uav_0/fsc_autopilot_ros2/activate_controller` (`ActivateController`) |
  | Live status | `/uav_0/fsc_autopilot_ros2/controller_type` (`std_msgs/String`, transient-local) |

  `ListControllers.Response` returns `ControllerInfo[]` — a struct per mode
  (`name`, `description`, `active`, `selectable`, `reason`), not parallel string arrays,
  so rows carry their own meaning. `ControllerInfo.name` is what `ActivateController`
  expects, and it matches the string published on `controller_type`, so a table row can
  be compared against live status without a translation table.

  **Scope: modes of the node that answers.** Baseline, CCM, CCM-outer and
  direct-actuation are separate executables chosen by the launcher, so a node can only
  report and activate what it switches into internally. Only the direct-actuation node
  implements this today (`SAFETY` / `DIRECT`); under the baseline stack both clients stay
  not-ready and the tab must report "unavailable" rather than erroring. Listing every
  workspace variant would need a supervisor that starts/stops processes — that does not
  exist, so do not present the table as if it did.

  Activation is idempotent (re-requesting the active mode succeeds as a no-op) and
  unknown names are rejected rather than defaulted. Responses always carry the true
  `active` mode, including on failure.

- The safe switch is deliberately **asymmetric**, and must stay that way: entering a mode
  that takes control from PX4 requires explicit confirmation and a selectable row, while
  `buttom_back_to_baseline` is one click, independent of the table selection, and is not
  disabled by a pending activation — an abort you have to confirm is an abort you cannot
  use. Activation timeouts must not claim the switch failed; the request may have been
  acted on, so re-read live status instead.
- Controller discovery/activation is separate from vehicle arm/disarm and PX4 flight-mode
  switching. It must not implicitly arm a vehicle or request OFFBOARD mode.

## Validation

There is currently no automated test suite, formatter, linter, or dependency manifest in the repository. Follow the surrounding style rather than reformatting entire files, and use the checks appropriate to the change:

```bash
# Parse/compile all Python sources without starting ROS or Qt
python3 -m compileall -q src

# Validate JSON configuration
python3 -m json.tool src/ROS_Node/geofence.json >/dev/null
python3 -m json.tool src/ROS_Node/slung_load.json >/dev/null

# Inspect the patch and accidental generated-file changes
git diff --check
git diff --stat
```

For GUI or ROS changes, also perform a manual smoke test in a sourced ROS 2 workspace. Confirm the window opens, the intended UAV tabs update, topic names, message fields, coordinate frames, units, and QoS match the running system, buttons publish/call only their intended interfaces, and shutdown does not leave the executor running. If `pyqtgraph` is unavailable, confirm that the single-drone application still opens and degrades gracefully.

Hardware-affecting controls such as arm, disarm, takeoff, land, emergency stop, and position commands must not be exercised on a live vehicle without explicit authorization and the normal flight-safety setup. Prefer simulation for validation.

## Change hygiene

- Check `git status --short` before and after editing; preserve unrelated and untracked user files.
- Keep changes scoped. This codebase has no enforced style tool, so follow the surrounding Python style instead of reformatting whole files.
- When adding a dependency or changing setup steps, document it in `README.md` (and add an appropriate dependency manifest if the task calls for packaging).
- Summarize which validation was run and clearly identify checks that require unavailable ROS messages, a display server, simulation, or hardware.
