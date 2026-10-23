#!/usr/bin/env python3
"""Batch evaluation of find-object missions in sim, through the REAL ground station.

    # from the repo root, with the Isaac sim, the baseline stack, the raw-VRPN relay and
    # isaacsim_vision_emulator running, the drone hovering, and NO other station flying it:
    FSC_GS_OBSTACLES=obstacles_sim.json QT_QPA_PLATFORM=offscreen PYTHONPATH=src \
        TRIALS=8 SEED=7 OUT=batch_eval.json /usr/bin/python3 test/sim/mission_batch_eval.py


N trials: place the red bottle at a random spot (mock camera), reset the drone to the start
through the station's own obstacle-aware follower, run the same mission, and record success,
position error against the true placement, time, looks, closest approach to each obstacle,
and whether the plan's viewpoints could see the bottle at all (offline, same camera model)."""
import json
import math
import os
import random
import sys
import time

from PyQt5 import QtWidgets
from PyQt5.QtCore import QEventLoop, QTimer

import rclpy
from std_msgs.msg import String
from GUI.single_drone_flight import Ui_MultiDroneGroundControlStation
from ROS_Node.ros_single_drone_control import SingleDroneRosThread, MISSION_SCAN_HEADINGS_DEG
from Common.obstacle_planner import Obstacle

sys.path.insert(0, os.environ.get('FSC_VISION_EMULATOR_SRC', os.path.expanduser(
    '~/Workspaces/fsc_autopilot_ws/src/isaacsim_vision_emulator')))
from isaacsim_vision_emulator.camera_model import CameraModel  # noqa: E402

N = int(os.environ.get('TRIALS', '8'))
SEED = int(os.environ.get('SEED', '7'))
OUT = os.environ.get('OUT', 'batch_eval.json')
OBSTACLES = [Obstacle('pillar', (0.5, 1.1, 0.8), (0.3, 0.3, 1.6)),
             Obstacle('chair', (-2.0, -2.5, 0.45), (0.5, 0.5, 0.9))]
LOCATIONS = [(2, 2, 1.2), (-2, 2, 1.2), (-2, -1.2, 1.2), (2, -2, 1.2)]
TASK = ("find the red bottle; if not in view check "
        + ", ".join(f"({x},{y},{z})" for x, y, z in LOCATIONS))
START = (0.0, 0.0, 1.0, 0.0)


def pump(seconds):
    loop = QEventLoop()
    QTimer.singleShot(int(seconds * 1000), loop.quit)
    loop.exec_()


def wait(cond, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        pump(0.05)
    return False


def quat(yaw):
    h = math.radians(yaw) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def predicted_visible(bottle):
    """Could any planned viewpoint + heading see it (camera geometry only)?"""
    cam = CameraModel()
    obj = {'label': 'bottle', 'position': list(bottle), 'size': [0.08, 0.08, 0.25]}
    vps = [START[:3]] + [tuple(map(float, l)) for l in LOCATIONS]
    return any(cam.observe(vp, quat(h), [obj]) for vp in vps for h in MISSION_SCAN_HEADINGS_DEG)


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QTabWidget()
    ui = Ui_MultiDroneGroundControlStation()
    ui.setupUi(win)
    gs = SingleDroneRosThread(ui)
    gs.start()
    win.show()
    mover = rclpy.create_node('batch_eval_mover')
    set_obj = mover.create_publisher(String, '/uav_0/mock_vision/set_object', 10)
    track = []
    rec = QTimer()
    def _rec():
        st = gs._drone_state()
        track.append(st[0] if st else None)
    rec.timeout.connect(_rec)
    rec.start(100)

    def reset_to_start():
        state = gs._drone_state()
        while state is None:
            pump(0.05)
            state = gs._drone_state()
        base = (*state[0], state[1])
        route = gs._start_route(base, START, "Batch eval reset")
        if isinstance(route, str):
            raise RuntimeError(f"cannot reset: {route}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 120:
            pump(0.1)
            st = gs._drone_state()
            if st is None:
                continue
            status = gs._route_step(route, st, "Batch eval reset")
            if status == 'arrived':
                return
            if status is not None:
                raise RuntimeError(f"reset failed: {status}")
        raise RuntimeError("reset timed out")

    pump(3.0)
    gs._toggle_llm_connection()
    wait(lambda: gs._llm_state == 'online', 30)
    ui.buttom_LLM_commit_2.setChecked(True)
    ui.buttom_LLM_auto.setChecked(True)
    pump(0.3)
    random.seed(SEED)
    results = []
    for trial in range(1, N + 1):
        while True:
            bottle = (round(random.uniform(-3.5, 3.5), 2), round(random.uniform(-3.5, 3.5), 2), 0.125)
            if all(math.dist(bottle[:2], o.center[:2]) >= 1.5 for o in OBSTACLES):
                break
        msg = String()
        msg.data = json.dumps({'name': 'red bottle', 'position': list(bottle)})
        set_obj.publish(msg)
        gs._object_memory = []          # every trial searches from scratch
        reset_to_start()
        pump(1.0)
        visible = predicted_visible(bottle)
        print(f"\n[trial {trial}/{N}] bottle at {bottle}, geometrically visible from the plan: {visible}",
              flush=True)
        ui.LLM_input.setPlainText(TASK)
        gs._send_llm_question()
        wait(lambda: gs._mission is not None and gs._mission['phase'] == 'confirm', 60)
        if gs._mission is None:
            print("   plan failed:", gs._mission_results[-1]['result'] if gs._mission_results else '?')
            results.append({'trial': trial, 'bottle': bottle, 'outcome': 'plan failed'})
            continue
        t0 = time.monotonic()
        start_i = len(track)
        gs._on_commit_clicked()
        wait(lambda: gs._mission is None, 900)
        elapsed = time.monotonic() - t0
        r = gs._mission_results[-1]
        found = r.get('found_room_xyz_m')
        err = math.dist(found[:2], bottle[:2]) if found else None
        seg = [p for p in track[start_i:] if p is not None]
        approach = {o.name: round(min(o.footprint_distance(x, y) for x, y, z in seg
                                      if o.blocks_altitude(z, 0.4)), 2)
                    if any(o.blocks_altitude(z, 0.4) for _, _, z in seg) else None
                    for o in OBSTACLES}
        row = {'trial': trial, 'bottle': bottle, 'predicted_visible': visible,
               'outcome': r['outcome'], 'found_xyz': found,
               'error_m': round(err, 3) if err is not None else None,
               'time_s': round(elapsed, 1), 'looks': len(r['looks']),
               'closest_approach_m': approach, 'result': r['result']}
        results.append(row)
        print(f"   -> {row['outcome']}  err {row['error_m']}  {row['time_s']} s  {row['looks']} looks  "
              f"closest {approach}", flush=True)
        json.dump(results, open(OUT, 'w'), indent=1)

    ok = [r for r in results if r['outcome'] == 'found']
    vis = [r for r in results if r.get('predicted_visible')]
    print("\n=== SUMMARY ===")
    print(f"trials {len(results)}; found+confirmed {len(ok)}; geometrically findable {len(vis)}; "
          f"found among findable {sum(1 for r in vis if r['outcome'] == 'found')}/{len(vis)}")
    if ok:
        print(f"position error: mean {sum(r['error_m'] for r in ok) / len(ok):.3f} m, "
              f"max {max(r['error_m'] for r in ok):.3f} m")
        print(f"time to find: mean {sum(r['time_s'] for r in ok) / len(ok):.0f} s, "
              f"max {max(r['time_s'] for r in ok):.0f} s; looks mean "
              f"{sum(r['looks'] for r in ok) / len(ok):.1f}")
    nf = [r for r in results if r['outcome'] == 'not found']
    if nf:
        print(f"not found: {len(nf)}, mean time {sum(r['time_s'] for r in nf) / len(nf):.0f} s")
    worst = {}
    for r in results:
        for k, v in (r.get('closest_approach_m') or {}).items():
            if v is not None:
                worst[k] = min(worst.get(k, 99), v)
    print(f"closest approach over all trials (margin 0.70 m): {worst}")
    json.dump(results, open(OUT, 'w'), indent=1)
    print(f"results: {os.path.abspath(OUT)}")
    os._exit(0)


if __name__ == "__main__":
    main()
