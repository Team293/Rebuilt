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
        boolean flywheel, intake, shooting, driving, held, braked, deployed, collected;
        int shots, legs, shotsCompleted, pickedLeg;
        double startedAt, firstShotAt, shotStartedAt, firstDriveAt, returnAt, secondShotSeconds;
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
        PathPlannerPath.clearCache();
        paths = PathPlannerAuto.getPathGroupFromAutoFile("Outpost Preload Collect Return");
        FlippingUtil.fieldSizeX = FieldConstants.fieldLength;
        FlippingUtil.fieldSizeY = FieldConstants.fieldWidth;
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    }

    private static JSONObject json(String path) throws Exception {
        return (JSONObject) new JSONParser().parse(Files.readString(Path.of(path)));
    }

    @Test
    void editorCompatibleAutoConnectsStoppedPathsAndReturnsAfterSecondPickup() throws Exception {
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
        assertTrue(end.getX() + halfLength < FieldConstants.LinesVertical.starting);
        assertEquals((FieldConstants.LinesHorizontal.rightBumpEnd + FieldConstants.Hub.nearRightCorner.getY()) / 2,
                end.getY(), 1e-6);
        assertEquals(0, finalPath.getGoalEndState().rotation().getDegrees(), 1e-6);
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
        double total = drive + 5;
        double pickupBegins = paths.get(0).getIdealTrajectory(config).orElseThrow().getTotalTimeSeconds()
                + 5 + paths.get(1).getIdealTrajectory(config).orElseThrow().getEvents()
                        .stream().mapToDouble(e -> e.getTimestampSeconds()).min().orElseThrow();
        System.out.printf("Second pickup starts %.3f s; final shot starts %.3f s, available %.3f s (deadline 20 s)%n",
                pickupBegins, total, 20-total);
        assertTrue(total < 19, "Return from both pickups with time for a second shooting attempt");
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
            assertFalse(state.intake, "Stop intake before each return trip");
        }
    }

    @Test
    void collectBeforeFirstShotOnBothTrenchReturnSides() {
        verifyCommandTree("Outpost Preload Collect Return", true);
        verifyCommandTree("Depot Preload Collect Return", true);
    }

    @Test
    void optimizedBumpVariantsCollectBeforeShootingAndCompleteBothSweeps() {
        verifyCommandTree("Outpost Double Pass Bump Return", true);
        verifyCommandTree("Depot Double Pass Bump Return", true);
    }

    private void verifyCommandTree(String name, boolean requireCompleteSweep) {
        SimHooks.pauseTiming();
        try {
            for (String cancelAt : new String[] {"none", "firstPickup", "secondPickup", "secondReturn", "secondShot"}) {
                boolean cancel = !cancelAt.equals("none");
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
                        if (state.legs == 1) {
                            state.firstDriveAt = Timer.getFPGATimestamp();
                            assertEquals(0, state.shots);
                        } else {
                            assertEquals(1, state.shotsCompleted);
                        }
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
                        if (!interrupted) state.returnAt = Timer.getFPGATimestamp();
                    }, () -> timer.hasElapsed(trajectory.getTotalTimeSeconds()), drive);
                }, () -> state.pose, pose -> state.pose = pose, true);
                NamedCommands.registerCommand("targetHub", Commands.none());
                NamedCommands.registerCommand("deployIntake", Commands.waitSeconds(1.5)
                        .andThen(Commands.runOnce(() -> state.deployed = true)));
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
                    assertFalse(state.intake);
                }, drive));
                NamedCommands.registerCommand("requestShooting2_5S",
                        Commands.runOnce(() -> fail("Collect-and-return autos must not shoot at the start")));
                double shotSeconds = 5.0;
                NamedCommands.registerCommand("requestShooting5S",
                            Commands.sequence(Commands.runOnce(() -> {
                                assertTrue(state.flywheel);
                                assertFalse(state.driving);
                                assertFalse(state.intake);
                                assertTrue(state.deployed && state.collected);
                                assertEquals(state.shots + 1, state.legs, "Each shot follows its collection return");
                                assertEquals(state.legs, state.pickedLeg, "Collect on each pass before shooting");
                                assertTrue(state.returnAt >= state.firstDriveAt);
                                state.shotStartedAt = Timer.getFPGATimestamp();
                                if (state.shots == 0) state.firstShotAt = state.shotStartedAt;
                                state.shooting = true;
                                state.collected = false;
                                state.shots++;
                            }), Commands.waitSeconds(shotSeconds)).finallyDo(() -> {
                                if (Timer.getFPGATimestamp() - state.shotStartedAt >= shotSeconds) state.shotsCompleted++;
                                if (state.shots == 2) state.secondShotSeconds = Timer.getFPGATimestamp() - state.shotStartedAt;
                                state.shooting = false;
                            }));
                NamedCommands.registerCommand("enableIntake", Commands.runOnce(() -> {
                    assertTrue(state.deployed, "Deployment must finish before pickup starts");
                    state.intake = true;
                    state.collected = true;
                    state.pickedLeg = state.legs;
                }));
                NamedCommands.registerCommand("disableIntake", Commands.runOnce(() -> state.intake = false));
                PathPlannerPath.clearCache();
                var auto = new PathPlannerAuto(name);
                auto.initialize();
                int ticks = 0;
                while (!auto.isFinished() && ticks < 1005) {
                    SimHooks.stepTiming(.02);
                    auto.execute();
                    ticks++;
                    if (cancelAt.equals("firstPickup") && state.intake && state.driving) break;
                    if (cancelAt.equals("secondPickup") && state.legs == 2 && state.intake && state.driving) break;
                    if (cancelAt.equals("secondReturn") && state.driving && state.pickedLeg == 2 && !state.intake) break;
                    if (cancelAt.equals("secondShot") && state.shots == 2 && state.shooting
                            && Timer.getFPGATimestamp() - state.shotStartedAt > .2) break;
                }
                assertTrue(state.firstDriveAt - state.startedAt < .1, "Leave immediately while deploying");
                if (cancel) {
                    assertFalse(auto.isFinished());
                    assertEquals(cancelAt.equals("firstPickup") ? 0 : cancelAt.equals("secondShot") ? 2 : 1,
                            state.shots, "Shooting must follow the completed return trips");
                } else {
                    assertTrue(auto.isFinished());
                    assertEquals(20, ticks * .02, .08);
                    assertEquals(2, state.shots, "Attempt shooting after both collection passes");
                    assertEquals(2, state.legs);
                    assertEquals(1, state.shotsCompleted, "The five-second return shot must finish");
                    assertTrue(state.secondShotSeconds > 1, "Final shot must get meaningful time before cutoff");
                    if (requireCompleteSweep) {
                        assertTrue(state.returnAt > state.firstShotAt, name + " must return after its second sweep");
                    }
                }
                auto.end(cancel);
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
            assertTrue(localY + extentY < FieldConstants.RightTrench.openingWidth
                    || (localY - extentY > FieldConstants.LinesHorizontal.rightBumpEnd
                            && localY + extentY < FieldConstants.Hub.nearRightCorner.getY()), pose.toString());
            assertEquals(red ? 180 : 0, Math.abs(pose.getRotation().getDegrees()), 1e-6);
        }
    }
}
