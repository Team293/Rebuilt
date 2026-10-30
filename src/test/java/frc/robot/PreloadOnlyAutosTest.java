package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import org.junit.jupiter.api.Test;

class PreloadOnlyAutosTest {
    private static class State {
        Pose2d pose = Pose2d.kZero;
        boolean flywheel, intake, deployed, shooting, driving, held, braked;
        int deployments, shots, paths;
        double shootingAt, stoppedShootingAt;
    }

    @Test
    void deploysAndShootsAtEachStartWithOnlyTheCenterAutoMoving() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        var config = RobotConfig.fromGUISettings();
        double startingX = FieldConstants.LinesVertical.starting - .8255 / 2;
        double bumpOffset = (FieldConstants.Hub.width + FieldConstants.LeftBump.width) / 2;
        var leftStart = new Pose2d(startingX, FieldConstants.fieldWidth / 2 + bumpOffset, Rotation2d.kZero);
        var rightStart = new Pose2d(startingX, FieldConstants.fieldWidth / 2 - bumpOffset, Rotation2d.kZero);
        String[] names = {"Center Back Up Preload", "Left Bump Preload", "Right Bump Preload"};
        SimHooks.pauseTiming();
        try {
            for (String name : names) {
                boolean center = name.startsWith("Center");
                var paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
                assertEquals(center ? 1 : 0, paths.size());
                var expected = center
                        ? new Pose2d(startingX - 1, FieldConstants.fieldWidth / 2, Rotation2d.kZero)
                        : name.startsWith("Left") ? leftStart : rightStart;
                if (center) {
                    var path = paths.get(0);
                    assertEquals(startingX, path.getStartingHolonomicPose().orElseThrow().getX(), 1e-6);
                    double lastX = startingX;
                    for (var sample : path.getIdealTrajectory(config).orElseThrow().getStates()) {
                        assertTrue(sample.pose.getX() <= lastX + 1e-6);
                        assertEquals(FieldConstants.fieldWidth / 2, sample.pose.getY(), 1e-6);
                        assertEquals(0, sample.pose.getRotation().getRadians(), 1e-6);
                        lastX = sample.pose.getX();
                    }
                    assertEquals(expected.getX(), lastX, 1e-6);
                    assertTrue(lastX - .8255 / 2 > FieldConstants.Tower.frontFaceX);
                }
                // Full run, cancellation while deploying, and cancellation while shooting.
                for (double cancelAt : new double[] {Double.POSITIVE_INFINITY, 1, 3}) {
                    var state = new State();
                    var drive = new SubsystemBase() {};
                    var shooter = new SubsystemBase() {};
                    var intake = new SubsystemBase() {};
                    AutoBuilder.resetForTesting();
                    AutoBuilder.configureCustom(path -> {
                        var trajectory = path.getIdealTrajectory(config).orElseThrow();
                        var timer = new Timer();
                        return new FunctionalCommand(() -> {
                            state.driving = true;
                            state.paths++;
                            timer.restart();
                        }, () -> state.pose = trajectory.sample(timer.get()).pose,
                                interrupted -> state.driving = false,
                                () -> timer.hasElapsed(trajectory.getTotalTimeSeconds()), drive);
                    }, () -> state.pose, pose -> state.pose = pose, true);
                    NamedCommands.registerCommand("targetHub", Commands.none());
                    NamedCommands.registerCommand("resetLeftBumpStart", AutoBuilder.resetOdom(leftStart));
                    NamedCommands.registerCommand("resetRightBumpStart", AutoBuilder.resetOdom(rightStart));
                    NamedCommands.registerCommand("deployIntake", Commands.sequence(
                            Commands.runOnce(() -> state.deployments++), Commands.waitSeconds(1.5),
                            Commands.runOnce(() -> state.deployed = true)));
                    NamedCommands.registerCommand("autoFlywheel", Commands.startEnd(
                            () -> state.flywheel = true, () -> state.flywheel = false, shooter));
                    NamedCommands.registerCommand("autoIntakeCleanup", Commands.startEnd(
                            () -> state.intake = false, () -> state.intake = false, intake));
                    NamedCommands.registerCommand("autoDriveCleanup", Commands.startEnd(
                            () -> {}, () -> state.braked = true));
                    NamedCommands.registerCommand("holdAutoPosition", Commands.run(() -> state.held = true, drive));
                    NamedCommands.registerCommand("stopFlywheel", Commands.runOnce(() -> state.flywheel = false));
                    NamedCommands.registerCommand("requestShooting2_5S", Commands.sequence(
                            Commands.runOnce(() -> {
                                assertTrue(state.deployed && state.flywheel);
                                assertFalse(state.driving || state.intake);
                                assertEquals(expected.getX(), state.pose.getX(), 1e-6);
                                assertEquals(expected.getY(), state.pose.getY(), 1e-6);
                                state.shots++;
                                state.shootingAt = Timer.getFPGATimestamp();
                                state.shooting = true;
                            }), Commands.waitSeconds(2.5)).finallyDo(() -> {
                                state.stoppedShootingAt = Timer.getFPGATimestamp();
                                state.shooting = false;
                            }));
                    double startedAt = Timer.getFPGATimestamp();
                    var auto = new PathPlannerAuto(name);
                    auto.initialize();
                    int ticks = 0;
                    while (!auto.isFinished() && ticks < 1005 && ticks * .02 < cancelAt) {
                        SimHooks.stepTiming(.02);
                        auto.execute();
                        ticks++;
                        assertFalse(state.intake, "Deploy the intake without collecting");
                    }
                    assertEquals(1, state.deployments);
                    if (Double.isInfinite(cancelAt)) {
                        assertTrue(auto.isFinished());
                        assertEquals(20, ticks * .02, .08);
                        assertEquals(1, state.shots);
                        assertEquals(center ? 1 : 0, state.paths);
                        assertTrue(state.held);
                        assertTrue(state.shootingAt - startedAt >= 2.5);
                        assertEquals(2.5, state.stoppedShootingAt - state.shootingAt, .04);
                        assertFalse(state.flywheel, "Stop the flywheel after shooting");
                    } else {
                        assertFalse(auto.isFinished());
                        if (cancelAt == 1) assertEquals(0, state.shots);
                        else assertTrue(state.shooting);
                    }
                    auto.end(!Double.isInfinite(cancelAt));
                    assertFalse(state.flywheel || state.intake || state.shooting || state.driving);
                    assertTrue(state.braked);
                }
            }
        } finally {
            AutoBuilder.resetForTesting();
            SimHooks.resumeTiming();
        }
    }
}
