"""Render motion clips to JSON for the 3D viewer.

The viewer does not re-implement any kinematics. This script runs the real
controller, records where every joint ended up at every frame, and writes
that out. If the Python changes, the animation changes -- there is no second
copy of the maths to drift out of sync.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from quadruped.robot import Quadruped, RobotConfig, BodyPose, LEG_NAMES
from quadruped.gait import GaitGenerator, GaitParams, Velocity, CRAWL, TROT
from quadruped.leg import Unreachable
from quadruped.vec import Vec3

FPS = 50                     # plenty smooth, and keeps the file small
DT = 1.0 / FPS


def record(robot: Quadruped, schedule, cmd: Velocity, seconds: float,
           params: GaitParams | None = None) -> list[dict]:
    """Run the controller and capture every frame."""
    gen = GaitGenerator(robot, schedule, params or GaitParams())
    frames = []
    for _ in range(int(seconds * FPS)):
        st = gen.step(DT, cmd)
        try:
            angles = robot.solve(st.pose, st.feet)
        except Unreachable:
            angles = robot.solve(st.pose, st.feet, clamp_to_limits=True)
        frames.append(frame(robot, st.pose, st.feet, angles, st.stance,
                            st.margin, st.phase))
    return frames


def frame(robot, pose, feet, angles, stance, margin, phase) -> dict:
    """One frame: everything the viewer needs to draw, already in world space."""
    r = pose.rotation
    legs = {}
    for name in LEG_NAMES:
        # joint_positions() returns body-frame points; lift them to the world
        # so the viewer never has to do a transform.
        pts = robot.legs[name].joint_positions(angles[name])
        legs[name] = {
            "pts": [[round(v, 4) for v in (r.apply(p) + pose.position)] for p in pts],
            "ang": [round(a, 4) for a in angles[name].as_tuple()],
            "down": name in stance,
        }
    return {
        "pos": [round(v, 4) for v in pose.position],
        "rpy": [round(pose.roll, 4), round(pose.pitch, 4), round(pose.yaw, 4)],
        "legs": legs,
        "margin": round(margin, 5) if margin != float("-inf") else None,
        "phase": round(phase, 4),
    }


def balance_demo(robot: Quadruped, seconds: float = 8.0) -> list[dict]:
    """Feet nailed to the floor while the body moves through every axis.

    This is the clip that shows what inverse kinematics actually buys you:
    nothing here commands a joint angle. Every frame says only "put the body
    here" and the IK works out all twelve. It is also exactly the motion a
    balance controller produces when it is holding the body level on ground
    that is tilting underneath it.
    """
    pose0, feet = robot.stand()
    frames = []
    n = int(seconds * FPS)
    for i in range(n):
        t = i / n
        # Four movements in sequence, each eased in and out so the transitions
        # do not snap.
        seg, u = divmod(t * 4, 1.0)
        ease = (1 - math.cos(2 * math.pi * u)) / 2       # 0 -> 1 -> 0
        roll = pitch = yaw = 0.0
        dz = dx = dy = 0.0
        if seg == 0:
            roll = math.radians(16) * math.sin(2 * math.pi * u)
        elif seg == 1:
            pitch = math.radians(16) * math.sin(2 * math.pi * u)
        elif seg == 2:
            yaw = math.radians(18) * math.sin(2 * math.pi * u)
            dy = 0.02 * math.sin(2 * math.pi * u)
        else:
            dz = -0.045 * ease

        pose = BodyPose(
            position=Vec3(pose0.position.x + dx, pose0.position.y + dy,
                          pose0.position.z + dz),
            roll=roll, pitch=pitch, yaw=yaw,
        )
        try:
            angles = robot.solve(pose, feet)
        except Unreachable:
            angles = robot.solve(pose, feet, clamp_to_limits=True)
        margin = robot.stability_margin(pose, feet)
        frames.append(frame(robot, pose, feet, angles, set(LEG_NAMES), margin, t))
    return frames


def main() -> None:
    robot = Quadruped()
    c = robot.config

    clips = {
        "balance": {
            "label": "Balance — feet planted, body moves",
            "note": "Nothing commands a joint angle. Every frame says only "
                    "“put the body here”; the IK solves all twelve.",
            "frames": balance_demo(robot),
        },
        "walk": {
            "label": "Walk forward — crawl gait",
            "note": "One foot up at a time, body leaning over the support "
                    "triangle before each lift. Statically stable throughout.",
            "frames": record(robot, CRAWL, Velocity(vx=0.06), 9.6),
        },
        "turn": {
            "label": "Turn in place",
            "note": "Same code as walking. Yaw rate alone, no forward speed — "
                    "the ω×r term makes each foot sweep its own arc.",
            "frames": record(robot, CRAWL, Velocity(omega=math.radians(30)), 9.6),
        },
        "arc": {
            "label": "Walk and turn together",
            "note": "Outside legs automatically take longer strides. No "
                    "dedicated turning code exists anywhere.",
            "frames": record(robot, CRAWL, Velocity(vx=0.06, omega=math.radians(20)), 9.6),
        },
        "strafe": {
            "label": "Strafe sideways",
            "note": "Pure lateral command. The legs never change role — only "
                    "the velocity vector does.",
            "frames": record(robot, CRAWL, Velocity(vy=0.05), 9.6),
        },
        "trot": {
            "label": "Trot — dynamic, diagonal pairs",
            "note": "Two feet down means the support polygon is a line: there "
                    "is no statically stable pose at all. Fast, and always "
                    "falling. Needs a balance controller to be real.",
            "frames": record(robot, TROT, Velocity(vx=0.12), 9.6),
        },
        "knees-fwd": {
            "label": "Knees forward \u2014 for comparison",
            "note": "The same walk with every knee mirrored to the front. "
                    "Identical stability; the difference is that the knee now "
                    "leads into whatever the robot walks toward, instead of "
                    "the foot reaching out ahead of it.",
            "frames": record(Quadruped(RobotConfig(knee_layout="forward")),
                             CRAWL, Velocity(vx=0.06), 9.6),
        },
        "unstable": {
            "label": "The same crawl with the lean switched off",
            "note": "shift_gain = 0. Identical gait otherwise. The centre of "
                    "mass leaves the support triangle and it tips — this is "
                    "what four legs do without a body shift.",
            "frames": record(robot, CRAWL, Velocity(vx=0.06), 9.6,
                             GaitParams(shift_gain=0.0)),
        },
    }

    data = {
        "fps": FPS,
        "robot": {
            "body_length": c.body_length,
            "body_width": c.body_width,
            "hip_offset": c.hip_offset,
            "thigh": c.thigh,
            "shank": c.shank,
            "stand_height": round(c.stand_height, 4),
            "mass": c.mass,
            "knee_layout": c.knee_layout,
        },
        "legs": list(LEG_NAMES),
        "clips": clips,
    }

    out = Path(__file__).resolve().parent / "viz" / "motion.json"
    out.write_text(json.dumps(data, separators=(",", ":")))
    kb = out.stat().st_size / 1024
    print(f"wrote {out}  ({kb:.0f} kB)")
    for k, v in clips.items():
        margins = [f["margin"] for f in v["frames"] if f["margin"] is not None]
        worst = f"{min(margins)*1000:+.1f} mm" if margins else "n/a (dynamic)"
        print(f"  {k:<10} {len(v['frames']):>4} frames   worst margin {worst}")


if __name__ == "__main__":
    main()
