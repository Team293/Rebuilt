# Visualize autos and paths in simulation

Run from the project folder using the WPILib JDK.

```sh
./gradlew simulateJava
```

In the simulator's NetworkTables/SmartDashboard controls, select an entry from
`Auto Path`. Full autos appear by name; every `.path` file also appears as
`Path: <name>`. Select Autonomous and enable the robot in the simulator's Driver
Station controls. Disable and re-enable to restart. Restart simulation after adding
new path files so the chooser discovers them.

To select and start a full auto directly:

```sh
./gradlew simulateJava '-Pauto=Outpost Preload Collect Return'
```

To select and start any individual path (omit the `.path` extension):

```sh
./gradlew simulateJava '-Ppath=Outpost Preload - Final Harvest'
```

These optional launch selections enable blue-alliance autonomous automatically and
disable after 20 seconds. Use either `-Pauto` or `-Ppath`. Add `-Pheadless` to hide
the simulator GUI. Stop the process with Ctrl+C before starting another simulator.

## AdvantageScope

1. Import `docs/sim/advantagescope-layout.json` using **File → Import Layout**.
2. In that window, choose **File → Connect to Simulator → NetworkTables 4
   (AdvantageKit)**. The address is `127.0.0.1`.
3. Start simulation. The layout displays the simulated robot pose and active path
   on the full-size 2026 field.

The layout uses `/RealOutputs/Robot/Pose` for the robot and
`/RealOutputs/Odometry/Trajectory` for the active path. Under
`/RealOutputs/Auto`, `Phase` describes driving, collecting, shooting, or waiting;
`IntakeRunning`, `ShootingRequested`, and `FlywheelRequested` expose the command
states. Add these to a Table or Line Graph tab to inspect their timing.

Each run saves a `.wpilog` in `logs/sim/`. Open one in AdvantageScope to scrub,
pause, and replay the entire run with the same layout. The drivetrain follows the
real command scheduler and CTRE simulation; shooter/intake flags show requested
behavior. Ball collection and projectile physics are not simulated.
