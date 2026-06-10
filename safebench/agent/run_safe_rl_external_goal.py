#!/usr/bin/env python3
"""
Run ChatScene/SafeBench safe-RL ego policy against ScenarioRunner-spawned hero.

Use case:
- CARLA server is running (e.g., docker `carlasim/carla:0.9.15`) on --port
- ScenarioRunner runs an OpenSCENARIO (.xosc) that spawns ego with role_name='hero' and assigns
  controller module 'external_control' (so ego is controllable externally).
- This script attaches to the spawned hero and drives it to a single goal pose using a pretrained
  safe-RL policy (PPO/SAC/TD3) from ChatScene-main SafeBench fork.

Important:
- Your Python version must match the CARLA Python API package you use (e.g. py3.7 egg -> python3.7).
- The policy expects the same 4D state as SafeBench env:
  state = [lateral_dis, -delta_yaw, speed(m/s), vehicle_front_flag]

Example:
  python3.7 ChatScene-main/safebench/agent/run_safe_rl_external_goal.py \\
    --host 127.0.0.1 --port 1403 --sync \\
    --algo td3 --weights ChatScene-main/safebench/agent/model_ckpt/safe_rl/td3_0/model_save/model.pt \\
    --goal "92,23,0,270"

  Add ``--viewer`` for a pygame chase camera + HUD (requires pygame; uses PythonAPI/examples/automatic_control.py).

  Add ``--debug-route`` to draw GlobalRoutePlanner init_waypoints in the simulator (world.debug).
  Use ``--debug-route-minimal`` for muted green dots only (no labels or arrows).

  Add ``--record`` with ``--record-outdir`` for FPV+BEV MP4 recording (requires ffmpeg).
  ``--viewer`` and ``--record`` are mutually exclusive. The script exits when the goal is reached.
"""

from __future__ import annotations

import argparse
import glob
import importlib.util
import math
import os
import sys
import time

import numpy as np
from typing import Any


def _add_carla_paths() -> None:
    """
    Add CARLA egg + PythonAPI to sys.path.
    This repo does not necessarily contain the egg under PythonAPI/carla/dist, so you may still
    need to set PYTHONPATH manually.
    """
    try:
        sys.path.append(
            glob.glob(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "..",
                    "..",
                    "PythonAPI",
                    "carla",
                    "dist",
                    "carla-*%d.%d-%s.egg"
                    % (
                        sys.version_info.major,
                        sys.version_info.minor,
                        "win-amd64" if os.name == "nt" else "linux-x86_64",
                    ),
                )
            )[0]
        )
    except IndexError:
        pass

    sys.path.append(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "PythonAPI",
            "carla",
        )
    )


def _repo_carla_root() -> str:
    """Path to the carla workspace root (parent of ChatScene-main)."""
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _load_automatic_control_module():
    """Load CARLA's automatic_control example (HUD, CameraManager, sensors)."""
    path = os.path.join(_repo_carla_root(), "PythonAPI", "examples", "automatic_control.py")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"automatic_control.py not found at {path}")
    spec = importlib.util.spec_from_file_location("carla_automatic_control_viewer", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load module spec from {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _ViewerWorldShim:
    """Minimal object satisfying automatic_control.HUD.tick(world, clock)."""

    def __init__(self, carla_world, carla_map, player, collision_sensor, gnss_sensor):
        self.world = carla_world
        self.map = carla_map
        self.player = player
        self.collision_sensor = collision_sensor
        self.gnss_sensor = gnss_sensor


def _find_heroes(world) -> list:
    vehicles = world.get_actors().filter("vehicle.*")
    heroes = [v for v in vehicles if v.attributes.get("role_name") == "hero"]
    heroes.sort(key=lambda a: a.id)
    return heroes


def _wait_for_hero(client, timeout_secs: float, poll_interval: float = 1.0):
    """
    Poll until a vehicle with role_name='hero' exists.

    ScenarioRunner may spawn hero before heavy Python imports finish; the launcher
    can also report hero early while the scenario ends during import. Re-wait here
    after connecting to CARLA with only the API loaded.
    """
    deadline = time.monotonic() + timeout_secs
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        world = client.get_world()
        heroes = _find_heroes(world)
        if heroes:
            print(
                f"Attached to hero (id={heroes[0].id}) after {attempt} poll(s).",
                flush=True,
            )
            return heroes[0], world
        remaining = int(deadline - time.monotonic())
        print(
            f"Waiting for role_name=hero... ({remaining}s left)",
            flush=True,
        )
        time.sleep(poll_interval)
    raise RuntimeError(
        f"No vehicle with role_name='hero' found within {timeout_secs:.0f}s. "
        "Did ScenarioRunner spawn the ego as hero with external_control, and is the "
        "scenario still running?"
    )


def _parse_goal(goal: str) -> tuple[float, float, float, float | None]:
    parts = [p.strip() for p in goal.split(",")]
    if len(parts) not in (3, 4):
        raise ValueError("goal must be 'x,y,z' or 'x,y,z,yaw'")
    x, y, z = map(float, parts[:3])
    yaw = float(parts[3]) if len(parts) == 4 else None
    return x, y, z, yaw


def _goal_reached(
    hero_loc,
    hero_yaw_deg: float,
    goal_loc,
    goal_yaw_deg: float | None,
    dist_thresh: float,
    yaw_thresh_deg: float,
) -> bool:
    if hero_loc.distance(goal_loc) > dist_thresh:
        return False
    if goal_yaw_deg is None:
        return True
    delta = (hero_yaw_deg - goal_yaw_deg + 180.0) % 360.0 - 180.0
    return abs(delta) <= yaw_thresh_deg


def _acc_steer_to_control(carla_mod, acc_norm: float, steer_norm: float) -> Any:
    """
    Mirror SafeBench env mapping (continuous mode):
    - action in [-1,1]^2
    - acc scaled by acc_max=3.0, steer scaled by steering_max=0.3
    - acc -> throttle/brake mapping: throttle = acc/3 if acc>0 else 0; brake = -acc/8 if acc<0 else 0
    """
    acc_max = 3.0
    steering_max = 0.3

    acc = float(np.clip(acc_norm, -1.0, 1.0) * acc_max)
    steer = float(np.clip(steer_norm, -1.0, 1.0) * steering_max)

    if acc > 0:
        throttle = float(np.clip(acc / 3.0, 0.0, 1.0))
        brake = 0.0
    else:
        throttle = 0.0
        brake = float(np.clip(-acc / 8.0, 0.0, 1.0))

    return carla_mod.VehicleControl(throttle=throttle, steer=steer, brake=brake)


def _draw_global_route_debug(
    carla_mod,
    world,
    init_waypoints,
    goal_loc,
    *,
    step: int = 5,
    life_time: float = 0.0,
    minimal: bool = False,
) -> None:
    """
    Visualize init_waypoints from GlobalRoutePlanner using world.debug.

    life_time=0 keeps markers until the simulation ends (CARLA default for persistent debug).
    minimal=True draws only small muted-green dots (no labels, arrows, or goal marker).
    """
    if not init_waypoints:
        return

    debug = world.debug
    z_offset = 0.3 if minimal else 0.5
    step = max(1, int(step))

    indices = {0, len(init_waypoints) - 1}
    indices.update(range(0, len(init_waypoints), step))

    if minimal:
        dot_color = carla_mod.Color(0, 160, 0)
        dot_size = 0.1
        for i in sorted(indices):
            loc = init_waypoints[i].transform.location + carla_mod.Location(z=z_offset)
            debug.draw_point(loc, size=dot_size, color=dot_color, life_time=life_time)
        return

    goal_color = carla_mod.Color(255, 0, 0)
    start_color = carla_mod.Color(0, 255, 0)
    route_color = carla_mod.Color(255, 255, 0)

    for i in sorted(indices):
        wp = init_waypoints[i]
        loc = wp.transform.location + carla_mod.Location(z=z_offset)
        yaw = wp.transform.rotation.yaw

        if i == 0:
            color = start_color
            label = f"start {i}"
        elif i == len(init_waypoints) - 1:
            color = route_color
            label = f"route {i}"
        else:
            color = route_color
            label = f"{i}"

        debug.draw_point(loc, size=0.15, color=color, life_time=life_time)
        debug.draw_string(
            loc + carla_mod.Location(x=0.5, z=0.3),
            f"{label} ({loc.x:.1f},{loc.y:.1f}) yaw={yaw:.0f}",
            draw_shadow=False,
            color=color,
            life_time=life_time,
        )

        end = loc + carla_mod.Location(
            x=1.0 * math.cos(math.radians(yaw)),
            y=1.0 * math.sin(math.radians(yaw)),
        )
        debug.draw_arrow(loc, end, arrow_size=0.15, color=color, life_time=life_time)

    goal_wp_loc = goal_loc + carla_mod.Location(z=z_offset)
    debug.draw_point(goal_wp_loc, size=0.2, color=goal_color, life_time=life_time)
    debug.draw_string(
        goal_wp_loc + carla_mod.Location(x=0.5, z=0.3),
        f"goal ({goal_loc.x:.1f},{goal_loc.y:.1f},{goal_loc.z:.1f})",
        draw_shadow=False,
        color=goal_color,
        life_time=life_time,
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Control ScenarioRunner ego (hero) with ChatScene safe-RL PPO/SAC/TD3"
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument(
        "--algo",
        required=True,
        choices=["ppo", "sac", "td3"],
        help="Which safe-RL policy to use.",
    )
    ap.add_argument(
        "--weights",
        required=True,
        help="Path to model weights (model.pt).",
    )
    ap.add_argument(
        "--goal",
        required=True,
        metavar="X,Y,Z[,YAW]",
        help="Goal pose in world coordinates (meters, degrees).",
    )
    ap.add_argument(
        "--sync",
        action="store_true",
        help="Use world.tick() each loop (synchronous mode).",
    )
    ap.add_argument(
        "--sleep",
        type=float,
        default=0.05,
        help="Sleep seconds per loop when not ticking (default: 0.05).",
    )
    ap.add_argument(
        "--route_resolution",
        type=float,
        default=1.0,
        help="Sampling resolution (m) for global route planner (default: 1.0).",
    )
    ap.add_argument(
        "--debug-route",
        action="store_true",
        help="Draw init_waypoints from GlobalRoutePlanner in the simulator (world.debug).",
    )
    ap.add_argument(
        "--debug-route-minimal",
        action="store_true",
        help="Draw route as muted green dots only (implies --debug-route; no labels or arrows).",
    )
    ap.add_argument(
        "--debug-route-step",
        type=int,
        default=5,
        help="Draw every Nth route waypoint when route debug is enabled (default: 5; always draws first/last).",
    )
    ap.add_argument(
        "--debug-route-life",
        type=float,
        default=0.0,
        help="Debug draw life_time in seconds (0 = persistent until sim ends).",
    )
    ap.add_argument(
        "--viewer",
        action="store_true",
        help="Open a pygame window with chase camera + HUD (like automatic_control.py).",
    )
    ap.add_argument(
        "--record",
        action="store_true",
        help="Record FPV+BEV to MP4 (mutually exclusive with --viewer; requires ffmpeg).",
    )
    ap.add_argument(
        "--record-outdir",
        metavar="DIR",
        help="Base output directory for recordings (required with --record).",
    )
    ap.add_argument(
        "--record-prefix",
        default=None,
        help="Recording filename prefix (default: safe_rl_<algo>).",
    )
    ap.add_argument(
        "--record-fps",
        type=int,
        default=15,
        help="Recording frame rate (default: 15).",
    )
    ap.add_argument(
        "--record-width",
        type=int,
        default=1280,
        help="Recording video width (default: 1280).",
    )
    ap.add_argument(
        "--record-height",
        type=int,
        default=720,
        help="Recording video height (default: 720).",
    )
    ap.add_argument(
        "--record-bev-height",
        type=float,
        default=50.0,
        help="BEV camera height above ego in meters (default: 50).",
    )
    ap.add_argument(
        "--goal-reached-dist",
        type=float,
        default=4.0,
        help="Stop when ego is within this distance (m) of goal (default: 4.0).",
    )
    ap.add_argument(
        "--goal-reached-yaw-deg",
        type=float,
        default=20.0,
        help="Yaw tolerance (deg) when goal includes yaw (default: 20).",
    )
    ap.add_argument(
        "--no-stop-at-goal",
        action="store_true",
        help="Keep running after goal is reached (legacy infinite loop).",
    )
    ap.add_argument(
        "--hero-wait-secs",
        type=float,
        default=120.0,
        help="Seconds to poll for role_name=hero after CARLA connect (default: 120).",
    )
    ap.add_argument(
        "--res",
        metavar="WIDTHxHEIGHT",
        default="1280x720",
        help="Pygame window resolution when --viewer is set (default: 1280x720).",
    )
    args = ap.parse_args()

    if args.viewer and args.record:
        ap.error("--viewer and --record are mutually exclusive")
    if args.record and not args.record_outdir:
        ap.error("--record requires --record-outdir")
    if args.record_prefix is None:
        args.record_prefix = f"safe_rl_{args.algo}"

    viewer_width, viewer_height = [int(x) for x in args.res.split("x")]

    _add_carla_paths()

    try:
        import carla  # type: ignore
    except ModuleNotFoundError as e:
        raise SystemExit(
            "Could not import CARLA Python API. You must run with a Python version compatible with your CARLA egg/wheel\n"
            "and include the egg on PYTHONPATH.\n"
            f"Original error: {e}"
        ) from e

    # Connect and attach to hero before heavy SafeBench imports (avoids race with
    # ScenarioRunner teardown while YOLO/scenario stacks load).
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    hero, world = _wait_for_hero(client, args.hero_wait_secs)

    # Imports that depend on PythonAPI/carla being available
    # NOTE: Some CARLA PythonAPI packages don't ship GlobalRoutePlannerDAO. In CARLA 0.9.15 here,
    # GlobalRoutePlanner is constructed directly from the carla.Map.
    from agents.navigation.global_route_planner import GlobalRoutePlanner  # type: ignore

    from safebench.gym_carla.envs.misc import get_preview_lane_dis  # type: ignore
    from safebench.gym_carla.envs.route_planner import RoutePlanner  # type: ignore

    from safebench.agent.safe_rl.policy import PPO, SAC, TD3  # type: ignore

    EgoVideoRecorder = None
    if args.record:
        from safebench.agent.osc_ego_recorder import EgoVideoRecorder  # type: ignore

    x, y, z, goal_yaw = _parse_goal(args.goal)
    goal_loc = carla.Location(x=x, y=y, z=z)

    # Build a route from current hero location to goal.
    carla_map = world.get_map()
    grp = GlobalRoutePlanner(carla_map, float(args.route_resolution))
    route = grp.trace_route(hero.get_location(), goal_loc)  # list[(waypoint, RoadOption)]
    init_waypoints = [wp for (wp, _roadopt) in route]
    if not init_waypoints:
        # Fallback: at least include goal projected to road
        init_waypoints = [carla_map.get_waypoint(goal_loc, project_to_road=True, lane_type=carla.LaneType.Driving)]

    if args.debug_route or args.debug_route_minimal:
        _draw_global_route_debug(
            carla,
            world,
            init_waypoints,
            goal_loc,
            step=args.debug_route_step,
            life_time=args.debug_route_life,
            minimal=args.debug_route_minimal,
        )
        style = "minimal green dots" if args.debug_route_minimal else "full markers"
        print(
            f"Drew global route debug ({style}): {len(init_waypoints)} init_waypoints "
            f"(every {max(1, args.debug_route_step)}th + endpoints). "
            "Fly spectator near the route to see markers."
        )
        if args.sync:
            world.tick()

    # Local route planner used by SafeBench to compute waypoints + hazards.
    routeplanner = RoutePlanner(hero, buffer_size=12, init_waypoints=init_waypoints)

    # Instantiate safe-RL policy and load weights.
    # These policies require ego_state_dim/action_dim/action_limit in config.
    policy_config = {
        "ego_state_dim": 4,
        "ego_action_dim": 2,
        "ego_action_limit": 1.0,
        "hidden_sizes": [256, 256],
        "ac_model": "mlp",
        # minimal required keys for each algo (filled where needed)
    }

    # A minimal logger substitute; safe_rl policies call logger.log / logger.store in some places.
    class _NullLogger:
        def log(self, *a, **k):
            return

        def store(self, *a, **k):
            return

    logger = _NullLogger()

    if args.algo == "ppo":
        algo_cfg = dict(policy_config)
        algo_cfg.update({"clip_ratio": 0.2, "target_kl": 0.01, "train_actor_iters": 1, "train_critic_iters": 1, "actor_lr": 1e-4, "critic_lr": 1e-4, "gamma": 0.99})
        policy = PPO(algo_cfg, logger)
    elif args.algo == "sac":
        algo_cfg = dict(policy_config)
        algo_cfg.update({"alpha": 0.01, "gamma": 0.99, "polyak": 0.995, "actor_lr": 1e-4, "critic_lr": 1e-4, "num_q": 2})
        policy = SAC(algo_cfg, logger)
    else:  # td3
        algo_cfg = dict(policy_config)
        algo_cfg.update({"act_noise": 0.0, "target_noise": 0.2, "noise_clip": 0.5, "policy_delay": 2, "gamma": 0.99, "polyak": 0.995, "actor_lr": 1e-4, "critic_lr": 1e-4, "num_q": 2})
        policy = TD3(algo_cfg, logger)

    # Load weights (binary torch file).
    policy.load_model(args.weights)

    recorder = None
    if args.record:
        assert EgoVideoRecorder is not None
        os.makedirs(args.record_outdir, exist_ok=True)
        recorder = EgoVideoRecorder(
            carla,
            world,
            hero,
            args.record_outdir,
            args.record_prefix,
            fps=args.record_fps,
            width=args.record_width,
            height=args.record_height,
            bev_height=args.record_bev_height,
        )

    ac_mod = None
    pg = None
    display = None
    hud = None
    camera_manager = None
    collision_sensor = None
    lane_invasion_sensor = None
    gnss_sensor = None
    viewer_world = None
    clock = None
    keyboard = None
    recording_active = False

    try:
        if args.viewer:
            try:
                import pygame
            except ImportError as e:
                raise SystemExit(
                    "--viewer requires pygame. Install with: pip install pygame\n"
                    f"Original error: {e}"
                ) from e

            pg = pygame
            ac_mod = _load_automatic_control_module()
            pg.init()
            pg.font.init()
            display = pg.display.set_mode(
                (viewer_width, viewer_height),
                pg.HWSURFACE | pg.DOUBLEBUF,
            )
            hud = ac_mod.HUD(viewer_width, viewer_height)
            world.on_tick(hud.on_world_tick)
            camera_manager = ac_mod.CameraManager(hero, hud)
            camera_manager.set_sensor(0, notify=False)
            collision_sensor = ac_mod.CollisionSensor(hero, hud)
            lane_invasion_sensor = ac_mod.LaneInvasionSensor(hero, hud)
            gnss_sensor = ac_mod.GnssSensor(hero)
            viewer_world = _ViewerWorldShim(world, carla_map, hero, collision_sensor, gnss_sensor)
            clock = pg.time.Clock()

            class _HudOnly:
                def __init__(self, hud_ref):
                    self.hud = hud_ref

            keyboard = ac_mod.KeyboardControl(_HudOnly(hud))

        while True:
            if args.viewer:
                clock.tick()

            # SafeBench RoutePlanner returns waypoints as [[x,y,yaw], ...] and hazards.
            waypoints, _roadopt, _cur_wp, _tgt_wp, _red_light, vehicle_front = routeplanner.run_step()
            if len(waypoints) < 3:
                # Ensure preview index works
                waypoints = (waypoints + waypoints + waypoints)[:3]

            ego_trans = hero.get_transform()
            ego_x = ego_trans.location.x
            ego_y = ego_trans.location.y
            ego_yaw = ego_trans.rotation.yaw / 180.0 * np.pi

            lateral_dis, w = get_preview_lane_dis(waypoints, ego_x, ego_y, idx=2)
            yaw_vec = np.array([np.cos(ego_yaw), np.sin(ego_yaw)])
            delta_yaw = np.arcsin(np.cross(w, yaw_vec))

            v = hero.get_velocity()
            speed = float(np.sqrt(v.x**2 + v.y**2))

            state = np.array([lateral_dis, -delta_yaw, speed, float(vehicle_front)], dtype=np.float32)

            # Act (deterministic). safe_rl policies return (action, logp/value...) depending on algo.
            if args.algo == "ppo":
                action, _v, _logp = policy.act(state, deterministic=True)
            else:
                action, _logp = policy.act(state, deterministic=True, with_logprob=False)

            # Match RLAgent behavior: invert steer sign.
            acc_norm = float(action[0])
            steer_norm = float(-action[1])

            control = _acc_steer_to_control(carla, acc_norm=acc_norm, steer_norm=steer_norm)
            hero.apply_control(control)

            if recorder is not None and not recording_active:
                recorder.start()
                recording_active = True

            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
                time.sleep(args.sleep)

            if recording_active and recorder is not None:
                recorder.update_bev()

            if not args.no_stop_at_goal and _goal_reached(
                hero.get_location(),
                hero.get_transform().rotation.yaw,
                goal_loc,
                goal_yaw,
                args.goal_reached_dist,
                args.goal_reached_yaw_deg,
            ):
                print(
                    f"Goal reached (dist <= {args.goal_reached_dist}m"
                    + (f", yaw <= {args.goal_reached_yaw_deg}deg" if goal_yaw is not None else "")
                    + ").",
                    flush=True,
                )
                break

            if args.viewer:
                if keyboard.parse_events():
                    break
                hud.tick(viewer_world, clock)
                camera_manager.render(display)
                hud.render(display)
                pg.display.flip()
    finally:
        if recorder is not None and recorder.active:
            recorder.stop()
        if args.viewer:
            if camera_manager is not None and camera_manager.sensor is not None:
                camera_manager.sensor.destroy()
                camera_manager.sensor = None
            if collision_sensor is not None and collision_sensor.sensor is not None:
                collision_sensor.sensor.destroy()
                collision_sensor.sensor = None
            if lane_invasion_sensor is not None and lane_invasion_sensor.sensor is not None:
                lane_invasion_sensor.sensor.destroy()
                lane_invasion_sensor.sensor = None
            if gnss_sensor is not None and gnss_sensor.sensor is not None:
                gnss_sensor.sensor.destroy()
                gnss_sensor.sensor = None
            if pg is not None:
                pg.quit()


if __name__ == "__main__":
    main()

