package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import org.junit.jupiter.api.Test;

class DepotBackInShootAutoTest {
    private static class State {
        Pose2d pose = Pose2d.kZero;
        boolean flywheel, intake, shooting, deployed, held, braked;
        int legs, movingShots;
        double startedAt, shootingAt;
    }

    @Test
    void backsToDepotThenCreepsAndShootsUntilDeadlineOrCancellation() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        var config = RobotConfig.fromGUISettings();
        var paths = PathPlannerAuto.getPathGroupFromAutoFile("Depot Back In And Shoot");
        assertEquals(2, paths.size());
        var start = paths.get(0).getStartingHolonomicPose().orElseThrow();
        assertEquals(FieldConstants.LinesVertical.starting, start.getX() + .8255 / 2, 1e-6);
        assertEquals(FieldConstants.Depot.depotCenter.getY(), start.getY(), 1e-6);
        var approach = paths.get(0).getIdealTrajectory(config).orElseThrow();
        var creep = paths.get(1).getIdealTrajectory(config).orElseThrow();
        assertEquals(approach.sample(approach.getTotalTimeSeconds()).pose,
                paths.get(1).getStartingHolonomicPose().orElseThrow());
        for (int leg = 0; leg < paths.size(); leg++) {
            var trajectory = paths.get(leg).getIdealTrajectory(config).orElseThrow();
            double previousX = Double.POSITIVE_INFINITY;
            for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += .01) {
                var sample = trajectory.sample(t);
                assertTrue(sample.pose.getX() <= previousX + 1e-6, "Drive backward throughout");
                previousX = sample.pose.getX();
                assertEquals(start.getY(), sample.pose.getY(), 1e-6);
                assertEquals(0, sample.pose.getRotation().getRadians(), 1e-6);
                assertTrue(sample.pose.getX() - .8255 / 2 >= .10 - 1e-6);
                if (leg == 1) assertTrue(sample.linearVelocity <= .15 + 1e-6);
            }
        }
        System.out.printf("Depot approach %.3f s, shooting creep %.3f s%n",
                approach.getTotalTimeSeconds(), creep.getTotalTimeSeconds());

        SimHooks.pauseTiming();
        try {
            for (boolean cancelDuringCreep : new boolean[] {false, true}) {
                var state = new State();
                state.startedAt = Timer.getFPGATimestamp();
                var drive = new SubsystemBase() {};
                var shooter = new SubsystemBase() {};
                var intake = new SubsystemBase() {};
                AutoBuilder.resetForTesting();
                AutoBuilder.configureCustom(path -> {
                    var trajectory = path.getIdealTrajectory(config).orElseThrow();
                    var timer = new Timer();
                    return new FunctionalCommand(() -> {
                        state.legs++;
                        if (state.legs == 1) assertFalse(state.shooting);
                        else assertTrue(state.shooting && state.intake);
                        timer.restart();
                    }, () -> {
                        var sample = trajectory.sample(timer.get());
                        if (state.shooting && sample.pose.getX() < state.pose.getX()) state.movingShots++;
                        state.pose = sample.pose;
                    }, interrupted -> timer.stop(),
                            () -> timer.hasElapsed(trajectory.getTotalTimeSeconds()), drive);
                }, () -> state.pose, pose -> state.pose = pose, true);
                NamedCommands.registerCommand("targetHub", Commands.none());
                NamedCommands.registerCommand("deployIntake", Commands.waitSeconds(1.5)
                        .andThen(Commands.runOnce(() -> state.deployed = true)));
                NamedCommands.registerCommand("enableIntake", Commands.runOnce(() -> state.intake = true));
                NamedCommands.registerCommand("autoFlywheel", Commands.startEnd(
                        () -> state.flywheel = true, () -> state.flywheel = false, shooter));
                NamedCommands.registerCommand("autoIntakeCleanup", Commands.startEnd(
                        () -> state.intake = false, () -> state.intake = false, intake));
                NamedCommands.registerCommand("autoDriveCleanup", Commands.startEnd(
                        () -> {}, () -> state.braked = true));
                NamedCommands.registerCommand("requestShooting20S", Commands.startEnd(() -> {
                    assertTrue(state.flywheel && state.deployed);
                    assertEquals(1.15, state.pose.getX(), .001);
                    state.shootingAt = Timer.getFPGATimestamp();
                    state.shooting = true;
                }, () -> state.shooting = false).withTimeout(20));
                NamedCommands.registerCommand("holdAutoPosition", Commands.run(() -> {
                    state.held = true;
                    assertTrue(state.shooting && state.intake);
                }, drive));
                var auto = new PathPlannerAuto("Depot Back In And Shoot");
                auto.initialize();
                int ticks = 0;
                while (!auto.isFinished() && ticks < 1005) {
                    SimHooks.stepTiming(.02);
                    auto.execute();
                    ticks++;
                    if (cancelDuringCreep && state.movingShots >= 10) break;
                }
                assertTrue(state.shootingAt - state.startedAt >= 2.5);
                assertEquals(2, state.legs);
                assertTrue(state.movingShots >= 10, "Shooting must overlap backward motion");
                if (cancelDuringCreep) {
                    assertFalse(auto.isFinished());
                    assertTrue(state.shooting && state.intake);
                } else {
                    assertTrue(auto.isFinished());
                    assertEquals(20, ticks * .02, .08);
                    assertTrue(state.held);
                    assertEquals(.51275, state.pose.getX(), .001);
                }
                auto.end(cancelDuringCreep);
                assertFalse(state.flywheel);
                assertFalse(state.intake);
                assertFalse(state.shooting);
                assertTrue(state.braked);
            }
        } finally {
            AutoBuilder.resetForTesting();
            SimHooks.resumeTiming();
        }
    }
}
