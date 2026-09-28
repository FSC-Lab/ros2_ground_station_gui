# LLM integration: how it works and why

How the single-drone ground station lets an operator ask a language model about the
drone's status, the design decisions behind it, and what went wrong along the way.
Written as preparation for explaining the project in an interview. Code-level rules
live in AGENTS.md ("Single-drone LLM status assistant").

## The short version

**The model never touches ROS 2.** The ground-station GUI already subscribes to the
drone's topics. When the operator asks a question, the GUI turns the latest values into
a small JSON snapshot. It pastes that snapshot into the prompt and sends it over HTTP to a
model on a separate GPU machine. The answer streams back as text into a chat box.

Status answers are **read-only**: nothing the model says is turned into a command.
The second stage, the VLM camera search below, can *propose* small drone moves, but
only an operator's click sends one.

A 30-second pitch:

> The drone and the ground station talk ROS 2. I added a status assistant to the
> ground station: the operator types a question like "how is the WiFi and where is
> the drone?", and a 7B vision-language model answers in about a second. Instead of
> giving the model access to ROS, the GUI injects a timestamped snapshot of the
> telemetry it already has into each prompt. That keeps it simple, bounded, and
> strictly read-only. The model cannot command the drone, and it is told to say
> "not available" rather than guess.

## The pieces and where they run

```
Drone computer (Jetson Orin)       Ground station (PyQt5 + rclpy)             LLM host (GPU PC)
─────────────────────────────      ──────────────────────────────────        ─────────────────────
PX4 via uXRCE-DDS                  ROS executor thread (QThread)
state estimator (odom)      ─DDS─►   subscription callbacks
OptiTrack / VRPN            (lab     → CommonData (mutex) + receive time
system monitor (WiFi, CPU)   WiFi)            │
camera + YOLO detector                        │ on "Send"
                                   Qt GUI thread                                HTTP proxy :8080
                                     snapshot → prompt ──HTTP/NetBird──────►    (no auth; NetBird only)
                                     chat box ◄─── streamed tokens ────────     Ollama (loopback only)
                                                                                qwen2.5vl:7b, 100% GPU
```

- **Drone ↔ ground station**: ROS 2 (DDS) over the lab WiFi.
- **Ground station ↔ LLM host**: plain HTTP over NetBird, a WireGuard mesh VPN. The
  connection is peer-to-peer, measured at 2.4 ms. The LLM host exposes only a small
  proxy on its NetBird address; Ollama itself stays on loopback because it has no
  authentication.
- **Model**: `qwen2.5vl:7b` (Qwen2.5-VL, 7B parameters, 5.5 GB on the GPU). It is
  vision-capable, which leaves room for sending camera frames later.

## Data flow, step by step

1. **Subscribe.** The GUI's ROS node subscribes to around twenty topics, including:
   - PX4 vehicle status, attitude and battery;
   - estimator odometry;
   - controller state;
   - raw OptiTrack frames;
   - the drone computer's system monitor (WiFi, CPU);
   - camera detections.

   Callbacks run on the ROS executor thread. They store each value in a
   mutex-protected `CommonData` object, along with its receive time
   (`last_update`).
2. **Snapshot.** When the operator presses Send, the GUI thread copies that state into a
   JSON document of roughly 600 tokens (`_llm_telemetry_snapshot()`). Every source
   carries `age_s`, and a source never received says `"no data received"`. It also
   includes GUI-side facts that are not topics: the last position command the operator
   sent, and the last 8 flight-log events.
3. **Prompt.** The request has three parts:
   - a fixed system prompt with the rules: answer only from the snapshot, never invent
     numbers, what the units and frames are, and that the model cannot control the
     drone;
   - the last 6 question/answer pairs as plain text;
   - the new question, with the fresh snapshot in front of it.

   Old snapshots are deliberately *not* kept in the history. The model then never
   mixes stale numbers into a new answer.
4. **Send and stream.** `LlmClient` (`src/Common/llm_client.py`) POSTs to `/api/chat`
   with Qt's asynchronous `QNetworkAccessManager`. The reply streams back one JSON object
   per line (Ollama's NDJSON format), and tokens appear in the chat box as they arrive.

Resulting behaviour: answers in **0.7-1.5 s**. The model loads in about 3-4 s when
connecting, and reloads in 5-10 s after 5 idle minutes.

## Design decisions (the "why" questions)

### Context injection instead of tool calling or an LLM ROS node

There are three common ways to connect an LLM to a robot:

| Approach | How it works | Trade-off |
| --- | --- | --- |
| **Context injection** (chosen) | The app puts the relevant state into the prompt | Simple, bounded, read-only, easy to test. The model cannot fetch anything else, and each answer is a point in time |
| Tool / function calling (e.g. via MCP) | The model calls `get_position()` and similar tools that query ROS | The model decides what to look at, and it is the usual route to commands. It needs a tool-calling-capable model, more round trips, and a guarded path into ROS |
| LLM as a ROS node | The model process subscribes to topics itself | Needs ROS on the LLM host and DDS across the VPN, and couples the GPU machine to the flight network |

This stage is status only, so injection gives the most safety per line of code. The
model's entire view of the world is one JSON document we control.

### Read-only, and why commands are a separate problem

The model's output is only ever displayed. The LLM host's proxy also has an
`/api/parse` endpoint that turns sentences into structured commands, e.g. "go up 2
meters and left 1" becomes UP 2 + LEFT 1. It is deliberately **not** used here:

- it returns only what the model said. The geofence check and resolving targets against
  the current pose live elsewhere (`nl_commander`) and would not run on this path.
- the GUI already has a safety convention: entering a mode that takes control needs
  explicit confirmation, while going back to the safe controller is always one click.
  Any LLM command path would have to follow the same rule.

In the end-to-end test, "Arm the drone and fly to x = 2" was refused in text, because
the system prompt says the model can only report status.

### Keeping the model honest

- **"No data" instead of zeros.** The GUI's data holders start at zero. Without
  receive timestamps, a model would happily say "the drone is at (0, 0, 0)" before any
  odometry had arrived. Each source now records when it last arrived, and the snapshot
  says `"no data received"` when it never has. Tested: with no telemetry at all, the
  model said the armed state could not be determined.
- **Ages on everything**, so the model can say a value may be out of date.
- **Explicit semantics in the system prompt.** Two examples:
  - the PX4 pre-flight flag is false for whole normal flights on these vehicles, so the
    prompt says not to call that a fault on its own;
  - position is in the local mocap frame with z up.

These are soft guards: they make invented values unlikely, not impossible.

### Not freezing the GUI

The ground station has a history of lag during flight: one plotting call was once 49%
of the GUI thread (see `docs/gui_responsiveness.md`). So:

- **No blocking I/O on the GUI thread.** `QNetworkAccessManager` is asynchronous and
  signal-driven. It needs no worker thread, and it never touches rclpy. The ROS/Qt
  boundary rule here is two-way: the Qt thread must not call into rclpy at all.
- **Bounded lock wait.** The snapshot reads shared state with `tryLock(50)`, a wait of
  at most 50 ms, once per question and not per GUI tick.
- **Throttled streaming.** Tokens are buffered and written to the widget at most 10
  times a second, not once per token.
- **Timeouts.** Health checks time out after 5 s. Chat uses a 45 s *inactivity*
  timeout, which Qt resets whenever bytes arrive, so a long answer is fine but a stall
  is caught.

### Connection lifecycle and UX

- **Connect** runs a health check (`GET /api/health`). If the model is not loaded, the
  GUI preloads it with a tiny request so the first real question is fast. While
  connected, it re-checks health every 15 s.
- **The status label** shows not connected / connecting / loading model / online /
  unreachable, colour-coded.
- **Send turns into Stop** while an answer streams. Enter sends; Shift+Enter adds a
  line.
- **Missing label.** If the `Api_status` label is missing from the Designer file, the
  station still starts and prints a warning. A cosmetic widget should not stop a flight
  tool from launching.

### Network and security

- The proxy has **no authentication**. The NetBird access policy is the only gate, so it
  should allow only the ground-station machines to reach TCP 8080 on the LLM host.
- Every question sends a telemetry snapshot to the LLM host. That is fine on a private
  mesh, but it is data leaving the ground station.
- NetBird needs internet access to reach its coordination servers, so an offline flight
  network would break the link.

## Problems hit along the way (good interview stories)

1. **The proxy silently ignored parameters.** I probed it with `curl` before writing
   the client:
   - `"stream": false` still streamed;
   - `options: {num_predict: 1}` still produced 15 tokens.

   So the proxy forwards only `model` and `messages`. The client therefore always
   parses a stream and does not rely on temperature or length limits. Lesson: verify
   the real interface; do not trust the API you assume is behind it.
2. **Preloading the model.** Ollama loads a model when it receives an empty message
   list, but this proxy rejects that with HTTP 400. Preloading uses a tiny real
   request instead ("Reply with the single word OK."), and its answer is discarded.
3. **The model used markdown despite being told not to.** Long answers came back with
   `**bold**` and `-` lists, shown raw. The fix: show the stream raw, then replace the
   finished answer with its rendered form.
   - The first attempt anchored the answer's start with a saved `QTextCursor`. A unit
     test showed Qt moves such a cursor when a newline is inserted at its position.
   - The working version locates the answer as the last `len(answer)` characters of
     the log. It replaces them only if they match exactly, so it can never overwrite
     anything else.
   - That was checked with the log's line cap trimming old lines mid-answer, and with
     another message landing in the middle.
4. **Status messages interleaving with a streaming answer.** A periodic health-check
   message written mid-answer landed inside the answer text. Now nothing is written to
   the chat log while an answer streams; the status label shows it instead.
5. **Noise from aborted requests.** Reading a reply after Stop made Qt print
   "device not open". The client now checks the reply is still open before reading.
6. **"The drone is at 0, 0, 0."** This is the placeholder-zeros problem described
   above; per-source receive timestamps solved it.

## How it was tested

- **Client edge cases, against a local fake HTTP server:**
  - normal stream, including a last line with no trailing newline;
  - HTTP 400 with an error body;
  - stream cut before `done`;
  - 3 s stall against a short timeout;
  - Stop followed immediately by a new question.
- **End to end, against the real LLM server:**
  - the real GUI ran headless (Qt offscreen);
  - a recorded flight was replayed from a rosbag on an isolated ROS domain
    (`ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`), so nothing reached the lab network.
    Replay QoS was overridden where PX4 topics are transient-local live.
  - results: correct position, WiFi, battery and OptiTrack answers in 0.7-1.5 s; the
    arm request refused; Stop, Shift+Enter and Disconnect working; the exact prompt
    captured and inspected.
- **Degraded cases:** no telemetry at all, no `Api_status` label, and server
  unreachable.

## Numbers to remember

| | |
| --- | --- |
| Answer latency | 0.7-1.5 s |
| Model load | ~3-4 s when connecting; 5-10 s after 5 min idle |
| Generation speed | ~85 tokens/s (measured on the LLM host) |
| Snapshot size | ~600 tokens |
| History resent | last 6 question/answer pairs |
| Timeouts | health 5 s; chat 45 s of inactivity |
| Health polling | every 15 s while connected |
| Chat log bound | 2000 lines |
| Stream to widget | at most 10 updates/s |
| Ground station → LLM host | NetBird P2P, 2.4 ms |

## Stage 2: VLM camera search ("is there a bottle in view?")

Added 2026-09-27, one step beyond status answers: the model looks at the camera.

```
Operator: "Is there a bottle in view?"
  -> recognised in code as a camera request for "bottle"
     (the chat model's {"action": "look"} reply is only a fallback)
  -> GUI: /uav_0/snapshot/request ---ROS 2---> Orin detector process
     Orin: latest frame as JPEG (base64) + THAT frame's detections, each with a
           camera-frame and drone-body position        (one message, ~85 kB, ~160 ms)
  -> VLM (qwen2.5vl) sees the image: "a blue water bottle on the floor" + {"visible", "move"}
  -> GUI matches the detector's result by label -> body position
  -> room position = R(q) · p_body + t, using the /uav_0/mocap pose at request time
  -> "Found the bottle ... room x, y, z"  (about 1.7 s end to end)
  not found? -> if LLM Control is on: propose one move on the "Commit Action" button
             -> operator clicks it -> settle -> new snapshot ... at most 5 moves
```

**Design decisions:**

- **Split the work by what each part is good at.** The VLM judges the image, whether
  the object is there, and suggests where to move. The detector gives the metric 3D
  position, from depth. Code does the bookkeeping: label matching, transforms and
  safety checks.
- **One message for the image and its detections.** They can never be mismatched,
  and the base64 JPEG is exactly what the VLM API takes. Standard `std_msgs/String`
  JSON avoided building a custom message package on the Orin.
- **The snapshot is served from inside the detector process,** because only one
  process can open the RealSense. A background ROS executor thread answers requests,
  so the camera loop never waits.
- **Calibrated transforms, verified.** Camera → body comes from an OptiTrack/AprilTag
  calibration done on the Orin. Body → room comes from the `/uav_0/mocap` pose the
  calibration was solved against. Replaying the calibration's own 8 AprilTag views
  through the ground-station code put the tag within 2.3 cm RMS of its surveyed
  position. In the replayed flight, the found bottle came out at z = 0.05 m, i.e. on
  the floor, as it should.
- **Moves only with a human click, and only safe small ones.** The moves are a fixed
  set: turn 30 deg, 0.3 m steps, 0.2 m up or down. A move is only offered if the drone
  is armed, in OFFBOARD, yaw-aligned and has OptiTrack, and the target is inside the
  geofence. Everything is re-checked at the click, and nothing is sent if the drone
  moved meanwhile. The operator's controls are on the tab itself:
  - an **LLM Control** switch, off at every launch;
  - a **Commit Action** button that names the pending move;
  - Stop.

  There is no modal dialog, so the abort controls stay usable.

**Problems hit (stories):**

1. **The VLM called a plainly visible bottle "not visible".** Asked to judge
   visibility, pick the matching detector box by index and choose a move in one JSON
   reply, the 7B model answered from the detector list instead of the image. I first
   ruled out transport: the prompt-token count went from 26 to about 1060 with the
   image, so it was arriving. The fix was **describe first, then decide**: one
   sentence of what it sees, then only `{"visible", "move"}`, with the detector matched
   in code. After that it was reliably right. Lesson: small models do better with
   narrow questions, and anything deterministic should not be delegated to the model.
2. **Which pose?** The estimator's odometry lags `/uav_0/mocap` by up to 10 cm and 5
   deg in flight. The extrinsic was calibrated against mocap, so that is the pose to
   use, sampled on the ROS thread at request time.
3. **A frame subtlety in the calibration code.** A helper composed the extrinsic with
   the camera's depth-to-color offset. That is right for CAD values, but double-counts
   about 15 mm for this calibration, which was solved directly in the color frame.
   Using the matrix as-is is what the 2.3 cm check confirms.
4. **Conversation memory vs routing.** Storing the search's result as the model's
   reply made "did you find a person earlier?" come back as "no data". Keeping the
   model's actual `{"action": "look"}` reply in history, and passing results through
   the snapshot, fixed it.
5. **The model stopped looking.** With an earlier result in its snapshot, the model
   answered a repeat "look for a person" from memory 4 times out of 4; a stronger
   prompt fixed only 1 in 4. The fix was to route explicit camera requests in code
   with a few patterns, which also saves the routing call, about 1 s. The model still
   answers past-tense questions from memory. Lesson: use the model where judgment is
   needed, and code where a rule will do.

**Numbers:**

| | |
| --- | --- |
| Snapshot round trip | ~160 ms |
| Snapshot message | ~85 kB (62 kB JPEG, 640x480, quality 95) |
| Frame age | 15-40 ms |
| VLM verdict | ~1 s |
| Bottle found end to end | ~0.6-1.7 s (0.6 s once routing moved into code) |
| Moves per search | at most 5, each operator-confirmed |
| Room-position check | 2.3 cm RMS |

## Limitations and next steps

- **Point-in-time answers.** The model does not watch the drone; each question takes a
  new snapshot.
- **It sees only what the GUI subscribes to.** Adding knowledge means adding a
  subscription and a snapshot field.
- **The hallucination guards are soft.** A small evaluation set of questions with
  expected answers, run against recorded flights, would catch regressions when the
  prompt or model changes.
- **Camera search not yet flown.** It was tested live on the bench and with a replayed
  flight; the first flights should be with a safety pilot.
- **The search only localises detector classes.** A chair can be seen by the VLM but
  has no position. Open-vocabulary detection would lift that limit.
- **General commands** would follow the camera search's pattern: a constrained move
  set, the GUI's own geofence and pose checks, and operator confirmation, never
  straight from model output to a publisher.
- **Tool calling.** Moving to tools (e.g. MCP) would let the model fetch on demand
  instead of receiving everything every time.

## Likely interview questions

**Why not let the LLM fly the drone?** Model output is probabilistic text; flight
commands must be validated and bounded. The camera search shows the pattern I used:
- the model may only *pick* from a fixed set of small moves;
- the GUI computes the target from the live pose and checks the geofence, armed state,
  mode, yaw alignment and OptiTrack;
- nothing is proposed unless the operator has switched **LLM Control** on (it is off
  at every launch);
- each move needs a **Commit Action** click;
- the abort paths stay one click away.

**How do you stop it making things up?** It only sees a snapshot where every value has
an age and missing data is explicit, and the system prompt forbids guessing. That
reduces invention but does not prove it away, so the next step would be an evaluation
set.

**Why not make the LLM a ROS node or give it tools?** For read-only status, injecting
context is simpler, bounded and easier to test. It also keeps ROS off the GPU machine.
Tools become worth it once the model needs to choose what to look at, or to act.

**How does the GUI stay responsive?**
- Asynchronous HTTP on the GUI thread, with no blocking and no rclpy.
- A bounded 50 ms lock wait once per question.
- Stream updates throttled to 10 Hz.
- Timeouts on every request.

**What if the LLM server dies mid-flight?** The assistant fails on its own and nothing
else does:
- the status label turns red;
- the chat log shows the error;
- a failed answer triggers an immediate health re-check.

Flight telemetry, alarms and controls do not depend on it.

**How does the model see the camera?** On request, the Orin sends the latest frame,
base64 JPEG at 640x480 quality 95, in the same message as that frame's detections. The
GUI attaches it to the VLM request's `images` field. It is one ~85 kB message per
question, not a stream, so WiFi load is negligible.

**Why do you trust the room position?** Three reasons:
- the calibration it relies on was solved against the same pose source it uses;
- the ground-station code reproduces the calibration's own AprilTag check to 2.3 cm;
- a found object on the floor comes out at floor height.
