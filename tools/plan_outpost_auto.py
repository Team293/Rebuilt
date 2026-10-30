#!/usr/bin/env python3
"""Generate the outpost auto using an explicit, approximate fuel-contact model.

Run from any directory: python3 tools/plan_outpost_auto.py
Fuel locations are representative samples, not a prediction of an actual match.
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PP = ROOT / "src/main/deploy/pathplanner"
FIELD_LENGTH, FIELD_WIDTH = 16.541, 8.069
BALL_RADIUS = 5.91 * 0.0254 / 2
PEN_DEPTH, PEN_WIDTH, DIVIDER = 72 * 0.0254, 206 * 0.0254, 2 * 0.0254
MAX_SPEED, PICKUP_FRACTION = 3.70, 0.40
TRANSIT_SPEED, TRANSIT_ACCEL = 3.0, 8.0
PRELOADS, CAPACITY, FIRE_RATE, SPIN_UP = 8, 20, 4.0, 2.5
MAX_PLANNED_LOAD = 25  # User's approximate 20–25-ball collection allowance.
CONTACT_TARGET = 32  # Extra exposure for scattering; contacts are not captured balls.
AUTO_SECONDS = 20.0


def fuel_layout(rows):
    """Two equal halves; 12 columns, with ball centers inside the fuel pen."""
    cx, cy = FIELD_LENGTH / 2, FIELD_WIDTH / 2
    xs = []
    for left, right in [(cx - PEN_DEPTH / 2, cx - DIVIDER / 2),
                        (cx + DIVIDER / 2, cx + PEN_DEPTH / 2)]:
        xs += [left + BALL_RADIUS + (right - left - 2 * BALL_RADIUS) * i / 5
               for i in range(6)]
    ys = [cy - PEN_WIDTH / 2 + BALL_RADIUS
          + (PEN_WIDTH - 2 * BALL_RADIUS) * i / (rows - 1) for i in range(rows)]
    return [(x, y) for x in xs for y in ys]


def contacted(layout, x, y0, y1, half_width, half_length):
    """Balls touched by the north-facing intake bar during its straight sweep."""
    result = set()
    for i, (bx, by) in enumerate(layout):
        dx = max(x - half_width - bx, 0, bx - x - half_width)
        dy = max(y0 + half_length - by, 0, by - y1 - half_length)
        if math.hypot(dx, dy) <= BALL_RADIUS:
            result.add(i)
    return result


def choose_sweep(layouts, removed, x, y0, half_width, half_length, dense_ceiling=45):
    for step in range(200):
        y1 = y0 + 0.01 * step
        sets = [contacted(layout, x, y0, y1, half_width, half_length) - old
                for layout, old in zip(layouts, removed)]
        # Expose the intake to extra balls. Capture efficiency is not measured.
        if CONTACT_TARGET <= len(sets[0]) <= 40 and len(sets[1]) <= dense_ceiling:
            return y1, sets
    raise ValueError("No sweep fits the scattering contact budget")


def point(x, y):
    return {"x": x, "y": y}


def waypoint(anchor, prev=None, nxt=None):
    return dict(anchor=point(*anchor), prevControl=point(*prev) if prev else None,
                nextControl=point(*nxt) if nxt else None, isLocked=False, linkedName=None)


def constraints(v, a):
    return dict(maxVelocity=v, maxAcceleration=a, maxAngularVelocity=180.0,
                maxAngularAcceleration=360.0, nominalVoltage=12.0, unlimited=False)


def named(name):
    return dict(type="named", data=dict(name=name))


def group(kind, *commands):
    return dict(type=kind, data=dict(commands=list(commands)))


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def generate():
    settings = json.loads((PP / "settings.json").read_text())
    hw, hl = settings["robotWidth"] / 2, settings["robotLength"] / 2
    settings["maxDriveSpeed"] = MAX_SPEED
    # Keep the dynamics model consistent with generated/TunerConstants.java.
    settings["driveGearing"] = 8.142857142857142
    settings["driveWheelRadius"] = 1.95 * 0.0254
    write_json(PP / "settings.json", settings)
    layouts = [fuel_layout(30), fuel_layout(36)]  # 360 nominal; 432 dense sensitivity case.
    empty = [set(), set()]
    y0, first_x, final_x = 0.90, 7.85, 8.15
    first_y, final_y = 2.34, 3.12  # Visually extended 0.45 m after field review.
    first_sets = [contacted(layout, first_x, y0, first_y, hw, hl) for layout in layouts]
    final_sets = [contacted(layout, final_x, y0, final_y, hw, hl) - old
                  for layout, old in zip(layouts, first_sets)]
    start_x, lane_y = 4.0218614 - hl, 50.34 * 0.0254 / 2

    for terminal, x, end_y, name in [
        (False, first_x, first_y, "Outpost Preload - Middle Loop"),
        (True, final_x, final_y, "Outpost Preload - Final Harvest"),
    ]:
        wp = [waypoint((start_x, lane_y), nxt=(start_x + .8, lane_y)),
              waypoint((5.95, lane_y), (5.15, lane_y), (6.65, lane_y)),
              waypoint((x, y0), (x, y0 - .20), (x, y0 + .25)),
              waypoint((x, end_y), (x, end_y - .25),
                       None if terminal else (x, end_y + .25))]
        rots = [(1., 0.), (2., 90.), (3., 90.)]
        if not terminal:
            wp += [waypoint((5.95, lane_y), (6.75, lane_y), (5.15, lane_y)),
                   waypoint((start_x, lane_y), (start_x + .8, lane_y))]
            rots.append((4., 0.))
        else:
            # GoalEndState already supplies the terminal rotation.
            rots = [(p, r) for p, r in rots if p < len(wp) - 1]
        markers = [dict(name="Begin pickup", waypointRelativePos=1.8,
                        endWaypointRelativePos=None, command=named("enableIntake"))]
        if not terminal:
            markers.append(dict(name="Batch collected", waypointRelativePos=3.1,
                                endWaypointRelativePos=None, command=named("disableIntake")))
        write_json(PP / "paths" / f"{name}.path", dict(
            version="2025.0", waypoints=wp,
            rotationTargets=[dict(waypointRelativePos=p, rotationDegrees=r) for p, r in rots],
            constraintZones=[dict(name="40% pickup speed", minWaypointRelativePos=1.7,
                                  maxWaypointRelativePos=3.0 if terminal else 3.4,
                                  constraints=constraints(MAX_SPEED * PICKUP_FRACTION, 1.0))],
            pointTowardsZones=[], eventMarkers=markers,
            globalConstraints=constraints(TRANSIT_SPEED, TRANSIT_ACCEL),
            goalEndState=dict(velocity=0, rotation=90 if terminal else 0),
            reversed=False, folder=None, idealStartingState=dict(velocity=0, rotation=0),
            useDefaultConstraints=False))

    # A fixed 20-second deadline includes all stages, followed by cleanup.
    # The final harvest stays in midfield; it does not start a doomed return trip.
    sequence = group("sequential", named("targetHub"),
        group("parallel", named("deployIntake"), dict(type="wait", data=dict(waitTime=SPIN_UP))),
        named("requestShooting2_5S"),
        dict(type="path", data=dict(pathName="Outpost Preload - Middle Loop")),
        named("requestShooting5S"), named("stopFlywheel"),
        dict(type="path", data=dict(pathName="Outpost Preload - Final Harvest")),
        named("holdAutoPosition"))
    auto = dict(version="2025.0", command=group("sequential", group("deadline",
        dict(type="wait", data=dict(waitTime=AUTO_SECONDS)), sequence,
        named("autoFlywheel"), named("autoIntakeCleanup"), named("autoDriveCleanup"))),
        resetOdom=True, folder=None, choreoAuto=False)
    write_json(PP / "autos" / "Outpost Preload Collect Return.auto", auto)

    # Optimistic arc-length bound: out and back to the nearest possible contact,
    # with ONE acceleration/braking pair per trip. Do not assume a stop at fuel.
    distance = FIELD_LENGTH / 2 - PEN_DEPTH / 2 - math.hypot(hw, hl) - BALL_RADIUS - start_x
    length = 2 * distance
    loop_bound = (2 * math.sqrt(length / TRANSIT_ACCEL)
                  if length < TRANSIT_SPEED ** 2 / TRANSIT_ACCEL
                  else length / TRANSIT_SPEED + TRANSIT_SPEED / TRANSIT_ACCEL)
    fastest_two = SPIN_UP + PRELOADS / 5 + 2 * (loop_bound + CAPACITY / 5)
    conservative_two = SPIN_UP + PRELOADS / FIRE_RATE + 2 * (loop_bound + CAPACITY / FIRE_RATE)
    report = dict(
        assumptions=dict(preloads=PRELOADS, targetBatch=CAPACITY, safeFiringRate=FIRE_RATE,
                         spinUpSeconds=SPIN_UP, autoSeconds=AUTO_SECONDS,
                         fullSpeedMPS=MAX_SPEED, pickupSpeedMPS=MAX_SPEED * PICKUP_FRACTION,
                         intakeWidthMeters=hw * 2, ballDiameterMeters=BALL_RADIUS * 2,
                         preloadShootingSeconds=2.5, returnShootingSeconds=5.0,
                         targetBallContacts=CONTACT_TARGET,
                         captureEfficiency=None,
                         representativeFuelCounts=[len(p) for p in layouts]),
        firstSweep=dict(x=first_x, startY=y0, endY=first_y,
                        potentialContacts=[len(p) for p in first_sets]),
        finalSweep=dict(x=final_x, startY=y0, endY=final_y,
                        potentialFreshContacts=[len(p) for p in final_sets],
                        previouslyDisturbedIgnored=[len(contacted(l, final_x, y0, final_y, hw, hl) & old)
                                                 for l, old in zip(layouts, first_sets)]),
        optimisticTwoFullCyclesLowerBoundAt5BPS=fastest_two,
        conservativeTwoFullCyclesMinSeconds=conservative_two,
        limitation="Contacts are pickup opportunities, not captured balls. Previously touched fuel is excluded as captured or scattered. Capture efficiency is unknown; no hopper sensor is present.")
    write_json(ROOT / "docs/autos/outpost-planning-model.json", report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    generate()
