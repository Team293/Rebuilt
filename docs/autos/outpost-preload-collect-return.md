# Outpost Preload Collect Return

Select **Outpost Preload Collect Return** in the **Auto Path** chooser, or use
[the simulation instructions](../simulation.md) to watch the complete routine in
AdvantageScope.

The auto starts on the right/outpost side and immediately drives to midfield while
deploying the intake and spinning up the flywheel. It keeps the preloaded balls,
collects more, returns and shoots the combined load for 5 seconds, then drives
out for a second pickup, returns over the bump, and requests another 5-second
shot. There is no initial wait or preload-only shot. A 20-second deadline
stops the mechanisms and brakes the drive. Pickup zones limit speed to 1.48 m/s
(40% of 3.70 m/s). Transit acceleration and braking are capped at 8 m/s²
with a 3 m/s transit speed limit. Pickup zones retain their 1 m/s² acceleration
limit. Intermediate waypoints do not request stops; the shooting endpoints do.

The first sweep runs near x=7.85 m and reaches y=2.34 m. The final sweep shifts to
x=8.15 m and extends toward y=3.12 m, reaching a different strip of fuel. The first
return shot begins around 5.95 seconds, second pickup starts around 12.73 seconds,
and the second return finishes around 17.99 seconds in the model. That leaves
about 2 seconds for the final shot before the deadline interrupts it. The
flywheel remains spun up, and the intake stops before both shooting windows. Visually check the
sweeps in PathPlanner and the actual stops and motion in robot simulation.

There is no hopper counter. These paths provide approximate collection coverage;
the requested shooting windows use the supplied effective firing rate.

On blue, center the robot at **(3.6091, 0.6393) m, heading 0°**, front bumper on
the start line and facing midfield through the right/outpost trench. Red flips to
**(12.9319, 7.4297) m, heading 180°**. Leave **Auto/PathFlipped** false.
Odometry resets before driving. The robot holds its forward heading through
both trench crossings and turns after clearing the structure.

- `Outpost Preload - Middle Loop.path`: midfield collection and return.
- `Outpost Preload - Final Harvest.path`: second pickup and bump return for shooting.
- `Outpost Preload Collect Return.auto`: shooting windows, drive sequence,
  deadline, and cleanup, with a sequential root for the PathPlanner editor.

PathPlanner settings use the drivetrain's 1.95-inch wheel radius, 8.142857:1 gearing,
and 3.70 m/s maximum speed. Vision and targeting use the full-size welded layout.
