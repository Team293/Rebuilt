# Double-pass bump-return autos

Select **Outpost Double Pass Bump Return** or **Depot Double Pass Bump Return**
in **Auto Path**. These are new variants of the existing collect-return autos.
Depot mirrors Outpost across the field width; red uses the existing alliance flip.

![Route overview](double-pass-bump-return.svg)

Both variants retain the original command sequence: deploy and spin up for
2.5 seconds, shoot preloads for 2.5 seconds, collect, stop for a 5-second shot,
then collect again until the 20-second deadline. The first pass leaves through
the trench and returns over the adjacent bump. The second departure moves back
to the trench lane before crossing into neutral territory.

| Blue starting center, heading 0° | Outpost | Depot |
| --- | --- | --- |
| Initial preload pose (m) | (3.6091, 0.6393) | (3.6091, 7.4297) |
| Return shooting pose (m) | (3.4125, 2.5105) | (3.4125, 5.5585) |

Place the initial front bumper **on the starting line**. The hood-clearance
logic uses the actual oriented bumper footprint while stationary, so touching
the line does not block the preload shot. Movement or rotation enables the
conservative approach buffer and retraction lookahead again.

Bump crossing is limited to 1.5 m/s with 1.5 m/s² acceleration, and pickup keeps
the existing 1.48 m/s (40%) speed limit. These limits cover the actual bump
crossing and pickup sections, with small buffers at intake events. Outside
those sections, transit uses 3.7 m/s and the model's 11.76 m/s² traction ceiling;
PathPlanner's motor, gearing, 60 A current, traction, module-speed, and cornering
limits determine the achievable acceleration and braking. There is no extra
3 m/s or 8 m/s² transit cap. Validate these modeled limits on the robot.

The second departure now curves smoothly into the trench lane rather than
making a sharp corner. Bezier handles, transit anchors, the return shooting X,
and the bump-zone boundaries were searched against the configured dynamics.
Both pickup sweeps retain their original anchors and coverage.

| Modeled timing | Previous variant | Optimized variant |
| --- | --- | --- |
| First path | 6.66 s | 5.16 s |
| Both shots complete / second departure | 16.66 s | 15.16 s |
| Second intake begins | 18.87 s | 17.48 s |
| Full routine including setup and shots | 22.88 s | 19.84 s |

The optimized paths cover about 19.36 m. Command-sequence tests at 20 ms confirm
both full paths and both shooting windows complete before the 20-second cutoff,
then hold position. The deadline still stops the drive, intake, and flywheel.
The modeled margin is small; real bump traversal and tracking can consume it.

The search evaluated 1,463 candidates across six deterministic starting seeds.
This is the best result found within the searched route family, not a proof of
a global optimum. The [optimization report](double-pass-optimization.json)
records the model limits, winning parameters, distance, and timings.

```sh
./gradlew simulateJava '-Pauto=Outpost Double Pass Bump Return'
./gradlew simulateJava '-Pauto=Depot Double Pass Bump Return'
./gradlew optimizeDoublePass
```

Trajectory tests check both sides, stopped path connections, bumper clearance,
trench outbound and bump inbound crossings, pickup and bump speed limits,
shooting-pose hood clearance, preserved shooting commands, and the deadline budget.
They also check continuous tangents at interior waypoints, use of the available
transit speed, and completion of both sweeps in the actual command sequence.
