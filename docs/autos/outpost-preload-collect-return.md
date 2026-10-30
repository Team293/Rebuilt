# Outpost Preload Collect Return

Select **Outpost Preload Collect Return** in the **Auto Path** chooser, or use
[the simulation instructions](../simulation.md) to watch the complete routine in
AdvantageScope.

The auto starts on the right/outpost side, spins up for 2.5 seconds while deploying
the intake, shoots preloads for 2.5 seconds, drives a deeper midfield sweep, returns
and shoots for 5 seconds, then drives out for a final pickup. A 20-second deadline
stops the mechanisms and brakes the drive. Pickup zones limit speed to 1.48 m/s
(40% of 3.70 m/s). Transit acceleration and braking are capped at 8 m/s²
with a 3 m/s transit speed limit. Pickup zones retain their 1 m/s² acceleration
limit. Intermediate waypoints do not request stops; the shooting endpoints do.

The first sweep runs near x=7.85 m and reaches y=2.34 m. The final sweep shifts to
x=8.15 m and extends toward y=3.12 m, reaching a different strip of fuel. The shorter preload
shooting window leaves more time for the final pickup before the 20-second deadline. Visually check the
sweeps in PathPlanner and the actual stops and motion in robot simulation.

There is no hopper counter. These paths provide approximate collection coverage;
the requested shooting windows use the supplied effective firing rate.

On blue, center the robot at **(3.6091, 0.6393) m, heading 0°**, front bumper on
the start line and facing midfield through the right/outpost trench. Red flips to
**(12.9319, 7.4297) m, heading 180°**. Leave **Auto/PathFlipped** false.
Odometry resets before shooting. The robot holds its forward heading through
both trench crossings and turns after clearing the structure.

- `Outpost Preload - Middle Loop.path`: midfield collection and return.
- `Outpost Preload - Final Harvest.path`: final outward pickup.
- `Outpost Preload Collect Return.auto`: shooting windows, drive sequence,
  deadline, and cleanup, with a sequential root for the PathPlanner editor.

PathPlanner settings use the drivetrain's 1.95-inch wheel radius, 8.142857:1 gearing,
and 3.70 m/s maximum speed. Vision and targeting use the full-size welded layout.
