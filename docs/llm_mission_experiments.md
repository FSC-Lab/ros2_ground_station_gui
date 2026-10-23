# LLM find-object missions: experiment protocol

How to measure, on the real drone in the OptiTrack arena, how well the VLM tab's
find-object missions work: whether they find the object, how accurately they place it, how
long they take, how close they fly to obstacles, and whether the cloud VLM beats the local
one. The same measurements exist for Isaac Sim (`isaacsim_vision_emulator` plus the batch
script described at the end), so the two can be compared.

Read `AGENTS.md` ("Single-drone LLM status assistant", "VLM camera search") first for how
missions, routes and the safety gates work.

## What the sim says to expect (baseline, 2026-10-22)

These come from the mock camera, which is easier than reality: it draws the object with its
name written on it, detects anything whose centre is in the image out to 6 m, and has no
occlusion, lighting or motion blur.

- Search geometry, start + 4 viewpoints, 8x8 m floor: 4 headings see 88% of it, 6 headings
  98% (the shipped default). The room-search grid (9 points at 1.2 m) keeps 91% even if the
  real detection range is only 2.5 m.
- Obstacle clearance: closest approach 0.83 m to a box with a 0.7 m margin, over a full run.
- Batch results (8 random placements): 8/8 found and confirmed, 142 s mean / 306 s max,
  closest approach 0.85 m (pillar) and 0.84 m (chair); details in the sim section below.

The hardware runs answer what the sim cannot: the real detection range and false-detection
rate, how the VLM does on real images, and how well the flown paths respect the margin with
real tracking and wind from the props.

## Arena setup

1. **Obstacles.** For each box, make a Motive rigid body with markers on the lid, set its
   pivot to the top centre, stream it, and add it to `scripts/config/obstacles_lab.json`:
   `{"name": "<rigid body name>", "topic": "/vrpn_mocap/<name>/pose", "size": [x, y, z]}`.
   Measure `size` with a tape. Restart the station and check its `[Obstacles]` line lists
   every box as tracked. Moving a box during a run is allowed: routes re-plan.
2. **Objects to find.** Use classes the onboard YOLO knows (bottle, cup, backpack, chair,
   ...). Record each object's true position: put a small rigid body on it for the run, or
   place it on a mark measured with the OptiTrack wand. Keep a list of 8 placements per
   object, spread over the arena, including some the plan should NOT see from any viewpoint
   (behind a box, outside the viewpoints' reach) -- they test the "not found" path.
3. **Station config.** `fsc_lab_machine.conf`: `OBSTACLES_FILE="obstacles_lab.json"`,
   `VLM_BACKEND="local"` for the first block of trials, `"openrouter"` for the second (key
   in `~/.config/fsc_gs/openrouter_key`). The audit log goes to
   `~/.ros/fsc_gs_llm_audit/` -- note the session file name for each block.

## Safety for every trial

- Safety pilot with the transmitter, ready to take over. STOP LLM Control and typed "hold"
  are not a substitute for the pilot.
- LLM Control and Allow Auto Task both on only while a trial runs; STOP LLM Control after.
- Battery: missions refuse to start below 40% and head home below 25%; land and swap well
  before that on hardware.
- Do not stand inside the arena while a mission flies: legs are planned around the boxes in
  the file, not around people.

## Procedure

For each placement, in order:

1. Place the object, record its true position and the placement number.
2. Fly to the start point (e.g. `go to position 0, 0, 1` and Commit), facing yaw 0.
3. Start recording (the bag below), then type the SAME task every trial, e.g.
   `find the red bottle; if not in view check (2,2,1.2), (-2,2,1.2), (-2,-2,1.2), (2,-2,1.2)`,
   check the plan, Commit.
4. Let it finish. Note: outcome (found and confirmed / found, not confirmed / not found /
   stopped), the reported room position, anything odd.
5. Stop the bag. STOP LLM Control.

Run the 8 placements with `VLM_BACKEND="local"`, then the same 8 with `"openrouter"`.
Optional third block: `search the room for the <object>` on 4 of the placements.

## What to record

The topics from `rosbag_experiment_recording.md`, plus:

```bash
ros2 bag record --output llm_mission_<placement> \
  /uav_0/state_estimator/local_position/odom \
  /uav_0/mocap \
  /uav_0/fsc_autopilot_ros2/position_controller/reference \
  /uav_0/snapshot/request /uav_0/snapshot/response \
  /uav_0/detections \
  /vrpn_mocap/<each obstacle>/pose \
  /vrpn_mocap/<object rigid body, if any>/pose
```

`/uav_0/snapshot/response` carries the JPEG the VLM judged, so a wrong verdict can be looked
at afterwards. The station's audit log holds the model's raw replies, every route and the
reason for every setpoint; it is matched to the bag by wall-clock time.

## Metrics

Per trial, then per block (local vs cloud VLM):

| Metric | From |
| --- | --- |
| Success: found and confirmed at the standoff | chat / audit `mission_look` (verify) |
| Findable: object inside a viewpoint's view at some heading | placement + camera geometry |
| False find: "found" with the object elsewhere, or a different object | reported vs true position |
| Missed: findable but not found | the two above |
| Position error | reported room position vs true position (horizontal) |
| Time to find / to give up | audit `mission_start` to the end |
| Looks used | audit `mission_look` count |
| Closest approach to each box (centre to footprint) | odom vs `/vrpn_mocap/*` |
| VLM agreement with the detector | audit `vlm_reply` vs `vlm_request` detections |

Report success among FINDABLE placements separately from overall success: a miss on a
placement no viewpoint could see is a planning gap, not a perception failure.

## Sim counterpart

`isaacsim_vision_emulator` takes `/uav_0/mock_vision/set_object` to move an object at
runtime; `test/sim/mission_batch_eval.py` drives the real station offscreen through N random
placements (reset to the start through the station's own router between trials) and writes
the same metrics as JSON. The 2026-10-22 runs, seed 7, task
`find the red bottle; if not in view check (2,2,1.2), (-2,2,1.2), (-2,-1.2,1.2), (2,-2,1.2)`,
pillar and chair from `obstacles_sim.json`, 6 headings:

| | First run | After the fixes it found |
| --- | --- | --- |
| Found and confirmed | 6/8 | **8/8** (all 8 geometrically findable) |
| Time to find, mean / max | 106 s / 189 s | 142 s / 306 s (the two hard ones now finish) |
| Looks per find, mean | 10.7 | 14.4 |
| Closest approach, pillar / chair (margin 0.7 m) | 0.85 / **0.21** m | 0.85 / 0.84 m |

What the first run found, all fixed: a location 0.3 m over the chair's top was legal, so the
drone flew there and re-planned every tick as its altitude wobbled across the clearance line
(now: vertical tracking tolerance in planning, true limits when monitoring, clearance 0.4 m);
and the 5-minute limit cut a mission off one tick after it found the bottle (now 10 min, and
never after a find). Position error is 0.000 m in sim because the mock detector is exact --
it says nothing about the real one.
