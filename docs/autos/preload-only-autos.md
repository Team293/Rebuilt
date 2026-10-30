# Preload-only autos

All three autos deploy the intake, target the hub, spin up the flywheel for
2.5 seconds, shoot the eight preloads for 2.5 seconds, then stop the flywheel
and hold the drive until the 20-second deadline. Intake rollers stay off.
Deployment and spin-up overlap; the center auto also backs up during this time.

| Auto | Blue starting center (m), heading 0° | Movement |
| --- | --- | --- |
| Center Back Up Preload | (3.6091, 4.0345) | Back up 1 m, stop, shoot |
| Left Bump Preload | (3.6091, 5.5585) | Stay still and shoot |
| Right Bump Preload | (3.6091, 2.5105) | Stay still and shoot |

Place the front bumper on the starting line. The bump starts align with the
middle of each bump's width, on the alliance side of the line. Left and right
refer to the blue alliance driver's view. Red uses the existing alliance flip.

The stationary autos reset odometry through the named commands
`resetLeftBumpStart` and `resetRightBumpStart`; they have no movement paths.
This also gives them the correct poses in AdvantageScope. The center auto resets
from **Center Preload - Back Up**, which uses 3 m/s and 8 m/s² travel limits.

Select any of the three names in **Auto Path** after restarting simulation, or
use the [simulation setup](../simulation.md), for example:

```sh
./gradlew simulateJava '-Pauto=Center Back Up Preload'
./gradlew simulateJava '-Pauto=Left Bump Preload'
./gradlew simulateJava '-Pauto=Right Bump Preload'
```

Run one simulator at a time. The shooting window uses the supplied firing rate;
there is no ball-count sensor.
