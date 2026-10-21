"""Walking: deciding where to put the feet, and when.

WHY A GAIT IS NOT JUST "MOVE THE LEGS IN ORDER"

tests/test_robot.py measures something that shapes this entire file. Standing
square, the robot has a 90 mm stability margin. Lift one foot and the margin
becomes *zero* -- the centre of mass lands exactly on the edge of the triangle
the other three feet make. Dead on the tipping point.

So the naive gait -- pick up a foot, swing it forward, put it down, repeat --
falls over. Four legs are not automatically stable; they are stable only while
the centre of mass stays inside the feet that are down. There are two honest
ways out, and this module implements both:

  CRAWL  Shift the body over the support triangle *before* lifting each foot.
         Slow (only one foot off the ground at a time, and the body is always
         weaving), but statically stable: stop the robot at any instant and it
         stands there. This is the one to build first, because a static gait
         that fails just stops, rather than falling on the floor.

  TROT   Lift diagonal pairs together. Two feet down means the support polygon
         is a *line*, so there is no statically stable position at all -- the
         robot is always falling, and catching itself with the next footfall.
         Much faster and it is what real quadrupeds use, but it only works
         with a balance controller running underneath (see balance.py).

THE COMMAND INTERFACE

Everything is driven by three numbers:

    vx     forward speed      (m/s)
    vy     sideways speed     (m/s, + is left)
    omega  turning rate       (rad/s, + is left)

That triple is the standard quadruped command, and turning comes out of it for
free. A foot planted on the ground must, from the body's point of view, move
backwards along exactly the path the body moves forwards along. For a body
both translating and rotating, a point at body-frame offset r moves at

    v_foot = v_body + omega x r

so a foot far from the turn centre sweeps further than one near it. Feed that
into the stance path and the robot turns without a single line of dedicated
turning code -- the outside legs simply take longer strides. Set vx = 0 and
omega != 0 and it spins in place.

THE SWING PATH

When a foot is in the air it has to get from where it lifted off to where it
will touch down, and the shape of that path matters more than it looks.

The requirement is zero horizontal speed at touchdown. If the foot is still
moving sideways when it lands, it scuffs -- and since it is now carrying
weight, the scuff pushes the whole robot. Real robots skate across the floor
for exactly this reason.

A cycloid satisfies it exactly. With s in [0, 1] across the swing:

    horizontal:  s - sin(2 pi s) / (2 pi)     scaled to the step length
    vertical:    (1 - cos(2 pi s)) / 2        scaled to the step height

Differentiate the horizontal term and you get (1 - cos(2 pi s)) / 1, which is
zero at both s = 0 and s = 1. The foot eases off the ground and settles back
down with no sideways velocity at either end. It is also the curve a point on
a rolling wheel traces, which is a nice way to remember it: the robot's feet
move like the tread of a wheel that is not there.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .robot import Quadruped, BodyPose, LEG_NAMES
from .vec import Vec3, Rot, clamp, lerp


@dataclass(frozen=True)
class Velocity:
    """What the robot is being told to do. The output of a joystick, and
    later the output of the AI/vision layer."""

    vx: float = 0.0        # forward, m/s
    vy: float = 0.0        # left, m/s
    omega: float = 0.0     # yaw rate, rad/s (+ = turn left)

    @property
    def is_stopped(self) -> bool:
        return abs(self.vx) < 1e-6 and abs(self.vy) < 1e-6 and abs(self.omega) < 1e-6

    def __repr__(self) -> str:
        return (f"Velocity(vx={self.vx:+.3f} m/s, vy={self.vy:+.3f} m/s, "
                f"omega={math.degrees(self.omega):+.1f} deg/s)")


@dataclass(frozen=True)
class GaitSchedule:
    """Which feet are on the ground when.

    `duty_factor` is the fraction of each cycle a foot spends on the ground.
    Above 0.75 at least three feet are always down (statically stable is
    possible); at 0.5 exactly two are, which forces a dynamic gait.

    `offsets` place each leg's cycle relative to the others, in fractions of
    a cycle. The pattern of offsets is what makes a gait recognisable -- it is
    the difference between a walk, a trot and a bound.
    """

    name: str
    duty_factor: float
    offsets: dict[str, float]
    statically_stable: bool

    def leg_phase(self, name: str, phase: float) -> float:
        return (phase + self.offsets[name]) % 1.0

    def in_stance(self, name: str, phase: float) -> bool:
        return self.leg_phase(name, phase) < self.duty_factor

    def swing_progress(self, name: str, phase: float) -> float:
        """How far through its swing this leg is, 0 to 1. Only meaningful
        when the leg is not in stance."""
        p = self.leg_phase(name, phase)
        return (p - self.duty_factor) / (1.0 - self.duty_factor)

    def stance_feet(self, phase: float) -> set[str]:
        return {n for n in LEG_NAMES if self.in_stance(n, phase)}


# One foot in the air at a time, each lifting a quarter-cycle after the last.
#
# THE ORDER IS THE WHOLE GAME, and the intuitive choice is the wrong one.
#
# Alternating diagonally -- RL, FR, RR, FL -- looks best. It never lifts two
# neighbouring legs in a row, so each support triangle is as fat as possible.
# Measured, it cannot keep the robot up: worst margin -0.2 mm at its best
# tuning, meaning it tips.
#
# The reason is that the body has to *get* to each triangle. Consecutive
# diagonal lifts put consecutive lean targets on opposite corners, so the body
# must cross its own centre twice per cycle, at speed, and arrives late every
# time. Instrumenting one cycle showed the lean at (-19, -9) mm while its
# target was (+23, -30) mm -- on the wrong side of the robot entirely.
#
# Stepping down one side and then the other -- RL, FL, RR, FR -- gives the
# body somewhere to stay: it sits out to the right while both left legs step,
# then crosses once and sits out to the left. Same duty factor, same speed,
# same everything else. Worst margin +19.1 mm instead of -0.2 mm.
#
# This is the classic wave/creep gait, and this is why it is the classic one.
CRAWL = GaitSchedule(
    name="crawl",
    duty_factor=0.80,
    offsets={"FR": 0.0, "RR": 0.25, "FL": 0.5, "RL": 0.75},
    statically_stable=True,
)

# Diagonal pairs, swapping twice per cycle. Fast, efficient, and always
# falling -- needs balance.py underneath it.
TROT = GaitSchedule(
    name="trot",
    duty_factor=0.5,
    offsets={"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5},
    statically_stable=False,
)

GAITS = {"crawl": CRAWL, "trot": TROT}


@dataclass
class GaitParams:
    """Tuning knobs. These are the numbers you will spend the most time
    adjusting once there is real hardware."""

    cycle_time: float = 1.6        # seconds for one full gait cycle
    step_height: float = 0.040     # how high a swinging foot lifts, m
    body_height: float = 0.0       # 0 = use the robot's default stand height
    # How far the body leans toward the support triangle before lifting a
    # foot, as a fraction of the way there. Only used by static gaits.
    # 0 falls over; too much wastes range of motion. ~0.6 is a good start.
    # 1.0 puts the body exactly on the support centroid, which measured best
    # across every command (worst case +4.8 mm, vs -1.4 mm at 0.8). The cost
    # is body travel: the further it leans, the more leg workspace the lean
    # itself consumes, so do not read 1.0 as "more would be better".
    shift_gain: float = 1.0
    # How quickly the lean follows the support triangle, in seconds. The
    # triangle jumps the instant a foot lifts or lands; a real body has mass
    # and cannot. This is the first-order lag that smooths the jump.
    lean_tau: float = 0.12
    # How far ahead of the gait clock the lean aims, as a fraction of a cycle.
    #
    # Without this the lean is purely reactive: it only starts moving once a
    # foot has already left the ground, and since the lag is comparable to the
    # swing duration it never arrives before the foot lands again. The robot
    # spends the whole swing catching up, and the margin dips negative.
    #
    # Animals shift their weight *before* picking a foot up. Aiming at the
    # support triangle that is about to exist, rather than the one that does,
    # is how that looks in code.
    lean_lead: float = 0.05
    max_step_length: float = 0.09  # safety clamp on stride, m
    # Extra room demanded of every planned stride, as a fraction.
    #
    # A step is checked for reach at the instant it is planned, but it stays
    # on the ground for most of a cycle, and the hips keep moving the whole
    # time. So "reachable now" is not "reachable throughout".
    #
    # This used to be 0.25 and was carrying the body lean as well, which it
    # could not: on a crawl the lean swings 55-67 mm, against the 22 mm this
    # buys on a 90 mm stride. The lean is now BOUNDED and subtracted
    # explicitly (see `_lean_reserve`), so this is back to what the name
    # says -- ordinary margin for the things that are not modelled, like the
    # 5 ms of staleness in the lean and the servo not landing exactly where
    # it was asked.
    reach_safety: float = 0.08
    # How fast the command itself is allowed to change. A robot cannot step
    # from standstill to full speed in one control tick, and pretending it can
    # breaks the gait at t=0: with feet starting square and the crawl stepping
    # one leg at a time, the last leg in the order waits most of a cycle for
    # its first step, and a body already at full speed drags it past its reach
    # before it ever moves. Ramping fixes that, and it is what the motors
    # would force on you anyway.
    max_accel: float = 0.08          # m/s^2
    max_yaw_accel: float = 0.6       # rad/s^2


class GaitGenerator:
    """Turns a velocity command into body poses and foot positions over time.

    Call `step(dt, velocity)` in a loop. It returns everything the pose solver
    needs, and never commands a foot outside the leg's workspace.
    """

    def __init__(
        self,
        robot: Quadruped,
        schedule: GaitSchedule = CRAWL,
        params: GaitParams | None = None,
    ) -> None:
        self.robot = robot
        self.schedule = schedule
        self.params = params or GaitParams()

        self.phase = 0.0
        self.time = 0.0

        # TWO body poses, and the distinction matters -- getting it wrong cost
        # this gait a third of its speed.
        #
        #   _nominal  where the body would be if it walked in a straight line,
        #             integrated purely from the velocity command.
        #   body      _nominal plus the sideways lean over the support feet.
        #
        # The lean must never feed back into _nominal. If it does, every tick
        # drags the body toward the centroid of the planted feet -- which
        # always lags behind a robot that is moving -- and at 200 Hz that
        # becomes a continuous brake. The robot still walks, just far slower
        # than commanded, which looks like a tuning problem rather than the
        # structural mistake it is.
        height = self.params.body_height or robot.config.stand_height
        self._nominal = BodyPose(position=Vec3(0.0, 0.0, height))
        self.body = self._nominal
        self._lean = Vec3(0.0, 0.0, 0.0)     # current offset, world frame

        # Where each foot currently is in the world. Stance feet do not move;
        # swinging feet are interpolated along their cycloid.
        self.feet: dict[str, Vec3] = robot.default_stance(self.body)

        # Bookkeeping for the swing: where each foot left the ground, and
        # where it is aiming to land.
        self._liftoff: dict[str, Vec3] = dict(self.feet)
        self._touchdown: dict[str, Vec3] = dict(self.feet)
        self._was_stance: dict[str, bool] = {n: True for n in LEG_NAMES}

        # 1.0 when the last command was delivered in full, less when it had to
        # be scaled back. Worth surfacing: on hardware this is the difference
        # between "the robot is slow" and "the robot is being asked for more
        # than it has".
        self.command_scale: float = 1.0

        # The command actually being executed, which lags the requested one
        # while it ramps.
        self._cmd = Velocity()

    # -- the main loop ------------------------------------------------------

    def step(self, dt: float, cmd: Velocity) -> "GaitState":
        """Advance one control tick."""
        p = self.params

        # 0. Ramp toward the requested command, then clamp what is left to
        #    what the legs can actually sustain.
        #    The ramp tracks the REQUEST, not the clamped result. Feeding the
        #    clamped value back into the ramp makes the two agree by
        #    construction -- the command can never climb past the clamp, so
        #    `command_scale` reports 1.0 however far over the line the request
        #    was, and the one diagnostic that should say "you are asking for
        #    more than this robot has" quietly always says "fine".
        self._cmd = self._rate_limit(cmd, dt)
        #    Clamp against where the ramp is HEADED, not where it is. A foot
        #    planned now is not lifted off the ground again until a whole
        #    cycle later, and by then the ramp has let the body speed up --
        #    so a stride that fits today is dragged further than planned
        #    tomorrow. That is the entire startup transient: feet starting
        #    square, the command climbing under them, the leg running out
        #    before the gait has even settled.
        horizon = self._rate_limit(cmd, self.params.cycle_time)
        cmd, self.command_scale = self._clamp_command(self._cmd, horizon)

        # 1. Integrate the NOMINAL body through the world -- never the leaned
        #    one. The command is in the body's own frame ("forward"), so
        #    rotate it into world axes first.
        advanced = self._advance(self._nominal, cmd, dt)
        yaw, new_pos = advanced.yaw, advanced.position

        # 2. Advance the gait clock -- but only while actually moving. A
        #    stopped robot should stand still, not march in place.
        if not cmd.is_stopped:
            self.phase = (self.phase + dt / p.cycle_time) % 1.0
        self.time += dt

        self._nominal = BodyPose(
            position=Vec3(new_pos.x, new_pos.y, self._nominal.position.z), yaw=yaw
        )
        # 3. Lean the body over the support feet. Uses last tick's foot
        #    positions, which at 200 Hz is 5 ms stale -- far below anything
        #    the hardware could resolve.
        self.body = self._apply_body_shift(self._nominal, cmd, dt)

        # 4. Move the feet.
        #
        #    TWO POSES GO INTO THIS, and they answer different questions.
        #
        #      _nominal  decides WHERE a foot should land. It has to, because
        #                the lean is derived from where the feet are: plan
        #                footsteps from the leaned body instead and the two
        #                chase each other. Measured, that runs away at a
        #                steady 12.5 mm/s -- the lean reached 136 mm on a
        #                200 mm robot inside ten seconds, and the gait
        #                quietly lost a fifth of its commanded distance.
        #
        #      body      decides WHETHER the leg can get there, since that is
        #                the pose the hip is actually at when it reaches.
        #                Checking against _nominal instead lets through steps
        #                that straighten a knee past its stop at high speed.
        #
        #    One of these is a planning frame and the other is a physical
        #    constraint. They only look interchangeable.
        for name in LEG_NAMES:
            in_stance = self.schedule.in_stance(name, self.phase)

            if in_stance:
                # A planted foot does not move in the world. That is the
                # entire definition of stance, and it is why the robot moves
                # forward at all: the body advances past a stationary foot.
                if not self._was_stance[name]:
                    self.feet[name] = self._touchdown[name]
            else:
                if self._was_stance[name]:
                    # Just lifted. Decide now where this foot will land.
                    #
                    # AIM AT WHERE THE BODY WILL BE, NOT WHERE IT IS. The
                    # foot lands a whole swing phase from now, by which time
                    # the hip has moved on -- so a target chosen around the
                    # CURRENT neutral point lands that far behind the leg.
                    #
                    # A crawl swings for 0.2 of a cycle and hid this; a trot
                    # swings for 0.5 of one, and at 0.15 m/s the foot was
                    # touching down 60 mm BEHIND neutral and lifting off at
                    # 180 mm -- one whole stance-length of lag. The leg hit
                    # 244 mm of a 243 mm reach and the knee went past its
                    # stop, while command_scale still read 1.00 because the
                    # scaler was measuring a stride the gait was not making.
                    #
                    # Both poses move forward together. Checking a future
                    # target against the hip's CURRENT position reads as a
                    # leg stretched a swing-length in front of itself, and
                    # the bisection below shortens a stride that was fine --
                    # measured, that clawed back 29 of the 60 mm.
                    # The lean cannot be predicted, so carry the current
                    # one: it is an offset on top of the nominal pose and
                    # changes slowly next to a swing.
                    self._liftoff[name] = self.feet[name]
                    landing = self._advance(self._nominal, cmd, self._swing_time())
                    lean = self.body.position - self._nominal.position
                    self._touchdown[name] = self._plan_touchdown(
                        name, landing, landing.shifted(lean), cmd
                    )
                s = self.schedule.swing_progress(name, self.phase)
                self.feet[name] = self._cycloid(
                    self._liftoff[name], self._touchdown[name], clamp(s, 0.0, 1.0)
                )

            self._was_stance[name] = in_stance

        stance = self.schedule.stance_feet(self.phase)
        return GaitState(
            pose=self.body,
            feet=dict(self.feet),
            stance=stance,
            phase=self.phase,
            time=self.time,
            margin=self.robot.stability_margin(self.body, self.feet, stance),
        )

    # -- footstep planning --------------------------------------------------

    def _advance(self, pose: BodyPose, cmd: Velocity, dt: float) -> BodyPose:
        """Where `pose` ends up after `dt` seconds of `cmd`.

        Used twice: once per tick to move the body, and once per lift-off to
        work out where the body will BE when the swinging foot lands.
        """
        heading = Rot.from_rpy(0.0, 0.0, pose.yaw)
        world_v = heading.apply(Vec3(cmd.vx, cmd.vy, 0.0))
        p = pose.position + world_v * dt
        return BodyPose(position=Vec3(p.x, p.y, pose.position.z),
                        yaw=pose.yaw + cmd.omega * dt)

    def _rate_limit(self, want: Velocity, dt: float) -> Velocity:
        """Move the executed command toward the requested one at a bounded rate."""
        dv = self.params.max_accel * dt
        dw = self.params.max_yaw_accel * dt
        c = self._cmd
        return Velocity(
            clamp(want.vx, c.vx - dv, c.vx + dv),
            clamp(want.vy, c.vy - dv, c.vy + dv),
            clamp(want.omega, c.omega - dw, c.omega + dw),
        )

    def _clamp_command(self, cmd: Velocity,
                       horizon: Velocity | None = None) -> tuple[Velocity, float]:
        """Scale the whole command down until every leg can keep up.

        WHY SCALE THE COMMAND RATHER THAN SHORTEN INDIVIDUAL STEPS

        A crawl at duty 0.80 leaves each foot planted for 80% of a cycle --
        1.28 s at the default timing. The body keeps moving that whole time,
        so the faster the command, the further each leg is dragged before it
        gets to step again. Past some speed the leg simply runs out of arm.

        Measured on the default geometry, each axis alone manages a lot:
        0.184 m/s forward, 0.098 m/s sideways, 47 deg/s of yaw. But they are
        not independent -- they all spend the same leg's workspace, so a
        combined command binds far earlier. 0.10 m/s with a little strafe and
        25 deg/s of turn is individually well inside every limit and jointly
        impossible.

        Shortening each leg's stride on its own (which `_pull_into_workspace`
        does as a last resort) keeps the robot upright but quietly changes
        where it goes: the legs that got shortened stop pulling their weight
        and the robot drifts off the commanded heading. Scaling the whole
        command keeps the *direction* exact and only gives up speed -- the
        robot still goes where it was told, just slower. That is almost always
        the behaviour you want, and it is easy to explain to whatever is
        driving.
        """
        if cmd.is_stopped:
            return cmd, 1.0

        pose = self.body

        # Bisect the SCALE, not each leg's stride. Every leg's sweep depends
        # on the yaw rate as well as the speed, and scaling the command
        # scales both together -- so "how much command fits" is one question
        # with one answer, not four independent ones. The previous version
        # varied stride length with omega pinned at the full request, which
        # made a fast spin look impossible at any speed and clamped it to a
        # dead stop.
        probe = horizon if horizon is not None else cmd
        if self._fits_at(probe, pose, 1.0):
            return cmd, 1.0
        lo, hi = 0.0, 1.0                   # lo known-good, hi known-bad
        for _ in range(12):
            mid = (lo + hi) / 2
            if self._fits_at(probe, pose, mid):
                lo = mid
            else:
                hi = mid
        if lo <= 0.0:
            return Velocity(), 0.0
        return Velocity(cmd.vx * lo, cmd.vy * lo, cmd.omega * lo), lo

    def _fits_at(self, cmd: Velocity, pose: BodyPose, scale: float) -> bool:
        """Can every leg see out a whole stance at `scale` of this command?"""
        c = Velocity(cmd.vx * scale, cmd.vy * scale, cmd.omega * scale)
        safety = 1.0 + self.params.reach_safety
        cap = self.params.max_step_length
        r_inv = pose.rotation.inverse()
        n = self._sweep_samples(c.omega)

        for name in LEG_NAMES:
            leg = self.robot.legs[name]
            drag = self._foot_drag(name, c)
            if drag.norm() < 1e-9:
                continue
            # Where the foot is PUT DOWN: half the drag ahead of neutral,
            # clamped by the stride cap. Where it ENDS UP: a full drag
            # behind that, cap or no cap.
            #
            # EXCEPT FOR THE FIRST CYCLE. The robot starts with its feet
            # square -- at neutral, not half a stride ahead of it -- so the
            # first stance drags each leg a FULL stride backwards instead of
            # half. Twice the designed excursion, once, before the gait has
            # settled. Planning for it here costs a little speed for one
            # cycle and nothing afterwards.
            td = Vec3(0.0, 0.0, 0.0) if self.time < self.params.cycle_time \
                else drag * 0.5
            if td.norm() > cap:
                td = td.normalized() * cap
            nb = r_inv.apply(self._neutral_foot(name, pose) - pose.position)
            if not all(leg.can_reach(q) for q in self._sweep(
                    leg.origin, nb, td * safety, drag * safety, c.omega, n)):
                return False
        return True

    # -- where a planted foot actually goes ---------------------------------

    def _swing_time(self) -> float:
        """Seconds a foot spends in the air."""
        return self.params.cycle_time * (1.0 - self.schedule.duty_factor)

    def _lean_reserve(self) -> float:
        """Workspace the body lean will take away, in metres.

        THE LEAN IS NOT FREE. It is a genuine translation of the body, so
        every millimetre of it is a millimetre further that some planted
        foot sits from its hip. The reach check has to pay for it up front.

        Before this existed the crawl broke in a way that looked like
        nothing: strafing at 0.14 m/s, one leg hit its knee stop for 135 ms
        of every 1.6 s cycle, forever, while `command_scale` read 1.00. The
        foot was not moving at all -- it was planted. The BODY lurched 17 mm
        sideways in 30 ms as the support triangle changed under it, and
        pulled the hip out from over its own foot.

        Bounded rather than predicted: the lean aims at the centroid of
        whichever feet are in stance, so sweeping the schedule over one
        cycle and taking the largest centroid offset gives a number the
        lean cannot exceed. Conservative by the overshoot of the
        first-order lag, which is the safe side to be wrong on.
        """
        if not self.schedule.statically_stable:
            return 0.0                       # a trot's support is diagonal:
                                             # the centroid is the body centre
        c = self.robot.config
        home = {}
        for n in LEG_NAMES:
            leg = self.robot.legs[n]
            home[n] = (leg.origin.x, leg.origin.y + leg.side.value * c.hip_offset)
        worst = 0.0
        for i in range(64):
            st = self.schedule.stance_feet(i / 64.0)
            if not st:
                continue
            cx = sum(home[n][0] for n in st) / len(st)
            cy = sum(home[n][1] for n in st) / len(st)
            worst = max(worst, math.hypot(cx, cy))
        return worst * self.params.shift_gain

    def _foot_drag(self, name: str, cmd: Velocity) -> Vec3:
        """How far this foot travels relative to the body over one stance,
        in body-frame axes.

        Not negotiable: the foot is on the ground and the body is moving, so
        this is simply how far the leg gets dragged. Shortening where the
        foot is PUT DOWN does not shorten it -- only slowing the body does.
        """
        leg = self.robot.legs[name]
        r = Vec3(leg.origin.x,
                 leg.origin.y + leg.side.value * self.robot.config.hip_offset, 0.0)
        # v = v_body + omega x r, with omega along +Z.
        foot_v = Vec3(cmd.vx - cmd.omega * r.y, cmd.vy + cmd.omega * r.x, 0.0)
        return foot_v * (self.params.cycle_time * self.schedule.duty_factor)

    def _sweep(self, hip: Vec3, nb: Vec3, td: Vec3, drag: Vec3,
               omega: float, n: int) -> list[Vec3]:
        """The path a planted foot traces in the body frame, sampled.

        `nb` is the neutral foot and `td` the touchdown offset from it, both
        in the body frame; `drag` is the full stance travel. The foot starts
        at nb + td and ends at nb + td - drag.

        TWO THINGS THIS GETS RIGHT THAT TESTING THE TWO ENDS DOES NOT.

        The arc. Under a turn the body yaws while the foot is planted, so in
        the body frame the foot swings through an arc, and the arc bulges
        outside the chord between its ends. Measured before this existed: a
        pure 90 deg/s spin drove the knee up to 2 degrees past its hard stop
        on 501 of 2400 timesteps. Straight-line commands were fine, which is
        why it hid -- the arc degenerates to the chord when omega is zero.

        The asymmetry. `td` can be shortened (by the stride cap, or by the
        bisection in `_pull_into_workspace`) but `drag` cannot. Treating the
        sweep as symmetric about neutral therefore tests a lift-off point
        the foot never reaches and misses the one it does.
        """
        T = self.params.cycle_time * self.schedule.duty_factor
        reserve = self._lean_reserve()
        end = 1.0 + self._swing_overshoot()
        out = []
        for i in range(n + 1):
            v = end * i / n                 # 0 at touchdown, 1 at lift-off
            psi = -omega * (v - 0.5) * T    # body frame rotates with the body
            c, sn = math.cos(psi), math.sin(psi)
            ex = nb.x + td.x - v * drag.x
            ey = nb.y + td.y - v * drag.y
            x, y = c * ex - sn * ey, sn * ex + c * ey
            # Charge the lean: the hip may be this much further from the
            # foot by the time the robot is actually standing here.
            ox, oy = x - hip.x, y - hip.y
            m = math.hypot(ox, oy)
            if m > 1e-9 and reserve > 0.0:
                x += ox / m * reserve
                y += oy / m * reserve
            out.append(Vec3(x, y, nb.z))
        return out

    def _swing_overshoot(self) -> float:
        """How much FURTHER the foot gets from its hip after lift-off,
        as a fraction of one stance's drag.

        The cycloid starts flat on purpose -- a soft lift-off, no snatch at
        the servo. The body does not start flat. So for the first tenth of
        the swing the body out-runs the foot, and the leg keeps extending
        after it has left the ground. That overshoot is the LAST place the
        leg runs out, and the stance sweep alone never sees it: measured on
        a trot strafing at 0.26 m/s, the whole stance stayed inside 236 mm
        of a 243 mm reach and then early swing hit 245 mm.

        Closed form, because the cycloid is: the foot's horizontal progress
        is  h(s) = s - sin(2.pi.s)/(2.pi)  while the body's is linear in s,
        so the gap is  sin(2.pi.s)/(2.pi),  peaking at s = 1/4 with value
        1/(2.pi) of the whole swing's travel. Scaled into units of one
        stance drag, the speed cancels and only the duty factor is left.
        """
        duty = self.schedule.duty_factor
        return (1.0 - duty) / (2.0 * math.pi * duty)

    def _sweep_samples(self, omega: float) -> int:
        """Two points is enough for a straight line. An arc needs the middle
        as well, and more of it the further it bends."""
        swept = abs(omega) * self.params.cycle_time * self.schedule.duty_factor
        return (2 if swept < math.radians(2.0)
                else min(12, 2 + int(swept / math.radians(5.0)) * 2))

    def _neutral_foot(self, name: str, pose: BodyPose) -> Vec3:
        """Where this foot would sit if the robot were standing still at
        `pose` -- directly below its hip, on the ground."""
        leg = self.robot.legs[name]
        hip = pose.rotation.apply(leg.origin) + pose.position
        lateral = pose.rotation.apply(Vec3(0.0, leg.side.value * self.robot.config.hip_offset, 0.0))
        return Vec3(hip.x + lateral.x, hip.y + lateral.y, 0.0)

    def _plan_touchdown(self, name: str, pose: BodyPose, reach_from: BodyPose,
                        cmd: Velocity) -> Vec3:
        """Choose where a foot should land.

        The rule: aim for the point that leaves the foot's stance sweep
        centred under the hip. A foot is planted for `duty_factor * cycle`
        seconds, during which the body travels past it, so landing half that
        distance *ahead* of neutral means it ends up half that distance
        behind at lift-off -- symmetric, and it uses the leg's range evenly
        in both directions.

        This is also where turning is handled: the per-foot velocity includes
        the  omega x r  term, so the outside legs of a turn automatically
        plan longer strides.
        """
        p = self.params
        stance_time = p.cycle_time * self.schedule.duty_factor

        leg = self.robot.legs[name]
        # Offset of this foot from the body centre, in body-frame axes.
        r = Vec3(leg.origin.x, leg.origin.y + leg.side.value * self.robot.config.hip_offset, 0.0)
        # v = v_body + omega x r, with omega along +Z.
        foot_v = Vec3(cmd.vx - cmd.omega * r.y, cmd.vy + cmd.omega * r.x, 0.0)

        # Half a stance-worth of travel, clamped so a large command cannot ask
        # for a stride the leg physically cannot make.
        reach = foot_v * (stance_time / 2.0)
        if reach.norm() > p.max_step_length:
            reach = reach.normalized() * p.max_step_length

        world_reach = pose.rotation.apply(reach)
        neutral = self._neutral_foot(name, pose)
        target = Vec3(neutral.x + world_reach.x, neutral.y + world_reach.y, 0.0)

        # Last line of defence: never plan a step the IK cannot solve. The
        # target was chosen in the nominal frame; whether it is *reachable* is
        # judged from where the hip actually is.
        return self._cap_stride(target, neutral)

    def _cap_stride(self, target: Vec3, neutral: Vec3) -> Vec3:
        """Clamp a planned footfall to the configured stride cap.

        THIS USED TO SHORTEN THE STRIDE UNTIL THE LEG COULD REACH IT, and
        that was worse than useless. Putting the foot down closer to
        neutral does not slow the body down, so the drag over stance is
        unchanged -- the whole sweep just slides BACKWARDS. Every
        millimetre taken off the front was added to the back, and the leg
        ran out at lift-off instead of at touchdown. Removing the
        bisection took the crawl from 296 out-of-reach timesteps per
        20 seconds of strafing to zero.

        The centred stride is the optimum, and when it does not fit the
        only real remedy is less speed. That is `_clamp_command`'s job.
        """
        d = target - neutral
        cap = self.params.max_step_length
        if math.hypot(d.x, d.y) <= cap:
            return target
        k = cap / math.hypot(d.x, d.y)
        return Vec3(neutral.x + d.x * k, neutral.y + d.y * k, 0.0)

    def _cycloid(self, a: Vec3, b: Vec3, s: float) -> Vec3:
        """The swing path from a to b, at progress s in [0, 1].

        Horizontal and vertical are separate functions of s -- see the module
        docstring for why this particular pair.
        """
        h = s - math.sin(2 * math.pi * s) / (2 * math.pi)
        z = self.params.step_height * (1 - math.cos(2 * math.pi * s)) / 2
        return Vec3(lerp(a.x, b.x, h), lerp(a.y, b.y, h), lerp(a.z, b.z, h) + z)

    # -- keeping the robot over its feet ------------------------------------

    def _apply_body_shift(self, nominal: BodyPose, cmd: Velocity, dt: float) -> BodyPose:
        """Lean the body toward the feet that are carrying it.

        This is the fix for the zero-margin problem measured in
        tests/test_robot.py. Rather than a rule per leg, aim at the centroid
        of whichever feet are in stance: when three feet are down that point
        is inside the support triangle by construction, so the lean is always
        in the right direction and one piece of code covers every phase.

        THE LEAN IS DIAGONAL, AND IT HAS TO BE.  It is tempting to shift only
        sideways, since side-to-side is the narrow axis and the obvious way to
        tip. That is wrong here. Lift FL and the support triangle is FR/RL/RR,
        whose nearest edge is the FR-RL *diagonal* -- and it runs through the
        centre of mass. Escaping an edge means moving perpendicular to it, so
        the useful direction is diagonal too. Keeping only the sideways
        component throws away half the escape and the margin stays negative.
        (Measured: lateral-only gives -19.6 mm, full diagonal gives +19.1 mm.)

        SMOOTHED.  The support triangle changes discontinuously the instant a
        foot lifts or lands, so the raw target jumps. A first-order lag turns
        that step into a lean the body could actually perform, and the time
        constant is a real physical quantity you will re-tune on hardware.

        WHAT THIS DOES *NOT* DO is feed back into the nominal trajectory --
        see __init__. The lean is an offset added on top, so the body weaves
        around a straight path instead of braking against it.
        """
        if not self.schedule.statically_stable:
            target = Vec3(0.0, 0.0, 0.0)
        else:
            # Aim at the support triangle we are about to be standing on,
            # not the one we are standing on now.
            stance = self.schedule.stance_feet(
                (self.phase + self.params.lean_lead) % 1.0
            )
            if stance and not cmd.is_stopped:
                cx = sum(self.feet[n].x for n in stance) / len(stance)
                cy = sum(self.feet[n].y for n in stance) / len(stance)
                # Straight at the centroid, in the ground plane, scaled by
                # the gain. Both horizontal components are kept.
                g = self.params.shift_gain
                target = Vec3(
                    (cx - nominal.position.x) * g,
                    (cy - nominal.position.y) * g,
                    0.0,
                )
            else:
                target = Vec3(0.0, 0.0, 0.0)

        # First-order lag: alpha = 1 - exp(-dt/tau) is the exact discrete form,
        # and unlike a bare fraction-per-tick it behaves the same at any
        # control rate. Swapping 200 Hz for 50 Hz must not change how the
        # robot walks.
        alpha = 1.0 - math.exp(-dt / max(self.params.lean_tau, 1e-6))
        self._lean = Vec3(
            lerp(self._lean.x, target.x, alpha),
            lerp(self._lean.y, target.y, alpha),
            0.0,
        )

        return BodyPose(
            position=nominal.position + self._lean,
            roll=nominal.roll,
            pitch=nominal.pitch,
            yaw=nominal.yaw,
        )


@dataclass(frozen=True)
class GaitState:
    """One tick of output: everything needed to drive the robot and to draw it."""

    pose: BodyPose
    feet: dict[str, Vec3]
    stance: set[str]
    phase: float
    time: float
    margin: float

    def contacts(self) -> dict[str, bool]:
        """Which feet the gait believes are on the ground. On real hardware
        this is what you compare against the foot switches -- a mismatch means
        the ground is not where the robot thought it was."""
        return {n: (n in self.stance) for n in LEG_NAMES}
