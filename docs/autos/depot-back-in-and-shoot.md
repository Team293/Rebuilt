# Depot Back In And Shoot

Start on the line directly in front of the depot. On blue, the robot center is
**(3.6091, 5.9631) m, heading 0°**, facing midfield, with the front bumper on
the starting line. Normal alliance flipping handles red.

The robot backs straight toward the depot to x=1.15 m while the flywheel spins up
and intake deploys. It waits until both the approach and the 2.5-second spin-up
finish, then enables the intake and requests shooting continuously while creeping
backward at **0.15 m/s** with **0.30 m/s²** acceleration.

The creep ends at x=0.51275 m, leaving the rear bumper 0.10 m from the alliance
wall using the configured robot length. It holds position and continues intaking
and requesting shots until the 20-second deadline. There is no separate preload
shooting phase. The turret continues targeting the hub throughout.

The robot keeps heading 0° while x decreases; it does not turn around to enter the
depot. The two paths are **Depot Back In - Approach** and **Depot Back In - Shooting
Creep**. The approach uses 3 m/s and 8 m/s² limits with the intake off.

To view it with the [simulation setup](../simulation.md):

```sh
./gradlew simulateJava '-Pauto=Depot Back In And Shoot'
```

Restart simulation to discover the new **Auto Path** entry. The robot pose shows
the backward motion; `Auto/ShootingRequested` and `Auto/IntakeRunning` show the
overlapping requests. Stop, cancellation, and the deadline clean up the mechanisms
and brake the drive.
