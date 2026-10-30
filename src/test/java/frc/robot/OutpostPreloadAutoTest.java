package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class OutpostPreloadAutoTest {
    private static final double PICKUP_SPEED = 3.70 * 0.40;
    private static List<PathPlannerPath> paths;
    private static RobotConfig config;
    private static double halfLength, halfWidth;
    private static SimState state;

    private static class SimState {
        boolean flywheel, intake, shooting, driving, held, braked;
        int shots, legs, shotsCompleted;
        double startedAt, firstShotAt, shotStartedAt;
        Pose2d pose = Pose2d.kZero;
    }

    @BeforeAll
    static void loadAuto() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        config = RobotConfig.fromGUISettings();
        var settings = json("src/main/deploy/pathplanner/settings.json");
        halfLength = ((Number) settings.get("robotLength")).doubleValue() / 2;
        halfWidth = ((Number) settings.get("robotWidth")).doubleValue() / 2;
        assertEquals(3.70, ((Number) settings.get("maxDriveSpeed")).doubleValue(), 1e-6);
        // Path event wrappers retain these commands; each test supplies its own state.
        NamedCommands.registerCommand("enableIntake", Commands.runOnce(() -> state.intake = true));
        NamedCommands.registerCommand("disableIntake", Commands.runOnce(() -> state.intake = false));
        paths = PathPlannerAuto.getPathGroupFromAutoFile("Outpost Preload Collect Return");
        FlippingUtil.fieldSizeX = FieldConstants.fieldLength;
        FlippingUtil.fieldSizeY = FieldConstants.fieldWidth;
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    }

    private static JSONObject json(String path) throws Exception {
        return (JSONObject) new JSONParser().parse(Files.readString(Path.of(path)));
    }

    @Test
    void editorCompatibleAutoConnectsStoppedPathsAndEndsInMidfield() throws Exception {
        var auto = json("src/main/deploy/pathplanner/autos/Outpost Preload Collect Return.auto");
        assertEquals("sequential", ((JSONObject) auto.get("command")).get("type"));
        assertEquals(2, paths.size());
        Pose2d start = paths.get(0).getStartingHolonomicPose().orElseThrow();
        assertEquals(FieldConstants.LinesVertical.starting, start.getX() + halfLength, 1e-6);
        assertEquals(FieldConstants.RightTrench.openingWidth / 2, start.getY(), 1e-6);
        for (var path : paths) {
            assertEquals(start, path.getStartingHolonomicPose().orElseThrow());
            assertEquals(0, path.getIdealStartingState().velocityMPS(), 1e-6);
            assertEquals(0, path.getGoalEndState().velocityMPS(), 1e-6);
        }
        var loop = paths.get(0);
        assertEquals(0, loop.getPoint(loop.numPoints() - 1).position.getDistance(start.getTranslation()), 1e-6);
        assertEquals(0, loop.getGoalEndState().rotation().getDegrees(), 1e-6);
        var finalPath = paths.get(1);
        var end = finalPath.getPoint(finalPath.numPoints() - 1).position;
        assertTrue(end.getX() > FieldConstants.LinesVertical.neutralZoneNear);
        assertTrue(end.getY() > 2);
        assertEquals(90, finalPath.getGoalEndState().rotation().getDegrees(), 1e-6);
    }

    @Test
    void depotAutoReflectsAcrossFieldWidthWithTheSameSequenceAndTiming() throws Exception {
        var original = json("src/main/deploy/pathplanner/autos/Outpost Preload Collect Return.auto");
        var mirrored = json("src/main/deploy/pathplanner/autos/Depot Preload Collect Return.auto");
        assertEquals(original, new JSONParser().parse(
                mirrored.toJSONString().replace("Depot Preload", "Outpost Preload")));
        var depotPaths = PathPlannerAuto.getPathGroupFromAutoFile("Depot Preload Collect Return");
        assertEquals(paths.size(), depotPaths.size());
        for (int leg = 0; leg < paths.size(); leg++) {
            var originalTrajectory = paths.get(leg).getIdealTrajectory(config).orElseThrow();
            var depotTrajectory = depotPaths.get(leg).getIdealTrajectory(config).orElseThrow();
            assertEquals(originalTrajectory.getTotalTimeSeconds(), depotTrajectory.getTotalTimeSeconds(), 1e-6);
            for (double t = 0; t <= originalTrajectory.getTotalTimeSeconds(); t += .02) {
                var expected = originalTrajectory.sample(t);
                var actual = depotTrajectory.sample(t);
                assertEquals(expected.pose.getX(), actual.pose.getX(), 1e-6);
                assertEquals(FieldConstants.fieldWidth - expected.pose.getY(), actual.pose.getY(), 1e-6);
                assertEquals(0, actual.pose.getRotation().plus(expected.pose.getRotation()).getRadians(), 1e-6);
                assertEquals(expected.linearVelocity, actual.linearVelocity, 1e-6);
                var reflectedBack = new Pose2d(actual.pose.getX(),
                        FieldConstants.fieldWidth - actual.pose.getY(), actual.pose.getRotation().unaryMinus());
                checkClearance(reflectedBack, false);
                checkClearance(FlippingUtil.flipFieldPose(reflectedBack), true);
            }
        }
    }

    @Test
    void generatedTrajectoriesClearBothTrenchesAndReportShootingBudget() {
        double drive = 0;
        for (var path : paths) {
            var trajectory = path.getIdealTrajectory(config).orElseThrow();
            double seconds = trajectory.getTotalTimeSeconds();
            assertTrue(Double.isFinite(seconds) && seconds > 0.1);
            drive += seconds;
            System.out.printf("%s: %.3f s%n", path.name, seconds);
            for (var sample : trajectory.getStates()) {
                checkClearance(sample.pose, false);
                checkClearance(FlippingUtil.flipFieldPose(sample.pose), true);
            }
        }
        double total = 2.5 + 2.5 + drive + 5;
        double pickupBegins = 2.5 + 2.5 + paths.get(0).getIdealTrajectory(config).orElseThrow().getTotalTimeSeconds()
                + 5 + paths.get(1).getIdealTrajectory(config).orElseThrow().getEvents()
                        .stream().mapToDouble(e -> e.getTimestampSeconds()).min().orElseThrow();
        System.out.printf("Final pickup starts %.3f s; uncut path ends %.3f s (deadline 20 s)%n", pickupBegins, total);
        assertTrue(2.5 + 2.5 + paths.get(0).getIdealTrajectory(config).orElseThrow()
                .getTotalTimeSeconds() + 5 < 20, "Complete both shooting windows before the deadline");
    }

    @Test
    void pickupEventsEnforceSpeedAndStopBeforeReturnTravel() {
        for (int leg = 0; leg < paths.size(); leg++) {
            state = new SimState();
            var trajectory = paths.get(leg).getIdealTrajectory(config).orElseThrow();
            var events = new EventScheduler();
            events.initialize(trajectory);
            boolean pickedUp = false;
            for (double t = 0; t <= trajectory.getTotalTimeSeconds() + .01; t += .005) {
                events.execute(t);
                if (state.intake) {
                    pickedUp = true;
                    assertTrue(trajectory.sample(t).linearVelocity <= PICKUP_SPEED + .02,
                            "Intake active above 40% speed");
                }
            }
            events.end();
            assertTrue(pickedUp, "Each pickup path must enable the intake");
            if (leg == 0) assertFalse(state.intake, "Stop intake before return travel");
        }
    }

    @Test
    void commandTreeShootsPreloadsForTwoPointFiveSecondsAndReturnForFiveSeconds() {
        SimHooks.pauseTiming();
        try {
            for (boolean cancelAtPickup : new boolean[] {false, true}) {
                state = new SimState();
                state.startedAt = Timer.getFPGATimestamp();
                var shooter = new SubsystemBase() {};
                var intake = new SubsystemBase() {};
                var drive = new SubsystemBase() {};
                AutoBuilder.resetForTesting();
                AutoBuilder.configureCustom(path -> {
                    var trajectory = path.getIdealTrajectory(config).orElseThrow();
                    var timer = new Timer();
                    var events = new EventScheduler();
                    return new FunctionalCommand(() -> {
                        assertFalse(state.shooting);
                        assertFalse(state.intake);
                        state.driving = true;
                        state.legs++;
                        events.initialize(trajectory);
                        timer.restart();
                    }, () -> {
                        events.execute(timer.get());
                        var sample = trajectory.sample(timer.get());
                        state.pose = sample.pose;
                        if (state.intake) {
                            assertTrue(sample.linearVelocity <= PICKUP_SPEED + .02);
                        }
                    }, interrupted -> {
                        events.end();
                        timer.stop();
                        state.driving = false;
                    }, () -> timer.hasElapsed(trajectory.getTotalTimeSeconds()), drive);
                }, () -> state.pose, pose -> state.pose = pose, true);
                NamedCommands.registerCommand("targetHub", Commands.none());
                NamedCommands.registerCommand("deployIntake", Commands.waitSeconds(1.5));
                NamedCommands.registerCommand("autoFlywheel", Commands.startEnd(
                        () -> state.flywheel = true, () -> state.flywheel = false, shooter));
                NamedCommands.registerCommand("autoIntakeCleanup", Commands.startEnd(
                        () -> state.intake = false, () -> state.intake = false, intake));
                NamedCommands.registerCommand("stopFlywheel", Commands.runOnce(() -> state.flywheel = false));
                NamedCommands.registerCommand("autoDriveCleanup", Commands.startEnd(
                        () -> {}, () -> state.braked = true));
                NamedCommands.registerCommand("holdAutoPosition", Commands.run(() -> {
                    state.held = true;
                    assertFalse(state.flywheel);
                    assertTrue(state.intake);
                }, drive));
                for (double shotSeconds : new double[] {2.5, 5.0}) {
                    String commandName = shotSeconds == 2.5 ? "requestShooting2_5S" : "requestShooting5S";
                    NamedCommands.registerCommand(commandName,
                            Commands.sequence(Commands.runOnce(() -> {
                                assertTrue(state.flywheel);
                                assertFalse(state.driving);
                                assertFalse(state.intake);
                                assertEquals(state.shots == 0 ? 2.5 : 5.0, shotSeconds);
                                state.shotStartedAt = Timer.getFPGATimestamp();
                                if (state.shots == 0) state.firstShotAt = state.shotStartedAt;
                                state.shooting = true;
                                state.shots++;
                            }), Commands.waitSeconds(shotSeconds)).finallyDo(() -> {
                                if (Timer.getFPGATimestamp() - state.shotStartedAt >= shotSeconds) state.shotsCompleted++;
                                state.shooting = false;
                            }));
                }
                var auto = new PathPlannerAuto("Outpost Preload Collect Return");
                auto.initialize();
                int ticks = 0;
                while (!auto.isFinished() && ticks < 1005) {
                    SimHooks.stepTiming(.02);
                    auto.execute();
                    ticks++;
                    if (cancelAtPickup && state.intake && state.driving) break;
                }
                assertTrue(state.firstShotAt - state.startedAt >= 2.5);
                if (cancelAtPickup) {
                    assertTrue(state.intake && state.driving);
                    assertFalse(auto.isFinished());
                } else {
                    assertTrue(auto.isFinished());
                    assertEquals(20, ticks * .02, .08);
                    assertEquals(2, state.shots);
                    assertEquals(2, state.legs);
                    assertEquals(2, state.shotsCompleted, "Both shooting windows must finish");
                }
                auto.end(cancelAtPickup);
                assertFalse(state.flywheel);
                assertFalse(state.intake);
                assertFalse(state.shooting);
                assertFalse(state.driving);
                assertTrue(state.braked);
            }
        } finally {
            AutoBuilder.resetForTesting();
            SimHooks.resumeTiming();
        }
    }

    private static void checkClearance(Pose2d pose, boolean red) {
        double cos = Math.abs(pose.getRotation().getCos()), sin = Math.abs(pose.getRotation().getSin());
        double extentX = halfLength * cos + halfWidth * sin;
        double extentY = halfLength * sin + halfWidth * cos;
        assertTrue(pose.getX() - extentX >= 0 && pose.getX() + extentX <= FieldConstants.fieldLength, pose.toString());
        assertTrue(pose.getY() - extentY >= 0 && pose.getY() + extentY <= FieldConstants.fieldWidth, pose.toString());
        double near = FieldConstants.LinesVertical.starting, far = FieldConstants.Hub.farRightCorner.getX();
        if (red) {
            double blueNear = near;
            near = FieldConstants.fieldLength - far;
            far = FieldConstants.fieldLength - blueNear;
        }
        if (pose.getX() + extentX >= near && pose.getX() - extentX <= far) {
            double localY = red ? FieldConstants.fieldWidth - pose.getY() : pose.getY();
            assertTrue(localY + extentY < FieldConstants.RightTrench.openingWidth, pose.toString());
            assertEquals(red ? 180 : 0, Math.abs(pose.getRotation().getDegrees()), 1e-6);
        }
    }
}
