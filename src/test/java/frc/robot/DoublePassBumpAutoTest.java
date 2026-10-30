package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.FieldConstants;
import frc.robot.subsystems.targeting.FieldTargeting;
import java.nio.file.Files;
import java.nio.file.Path;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DoublePassBumpAutoTest {
    private static RobotConfig config;
    private static boolean intake;
    private static final double HALF_BUMPER = .8255 / 2;

    @BeforeAll
    static void setup() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        config = RobotConfig.fromGUISettings();
        NamedCommands.registerCommand("enableIntake", Commands.runOnce(() -> intake = true));
        NamedCommands.registerCommand("disableIntake", Commands.runOnce(() -> intake = false));
    }

    @Test
    void preservesTheExistingCommandSequenceAndShootingTimes() throws Exception {
        for (String side : new String[] {"Outpost", "Depot"}) {
            String original = Files.readString(Path.of("src/main/deploy/pathplanner/autos/"
                    + side + " Preload Collect Return.auto"));
            String variant = Files.readString(Path.of("src/main/deploy/pathplanner/autos/"
                    + side + " Double Pass Bump Return.auto"));
            variant = variant.replace(side + " Double Pass - Bump Return", side + " Preload - Middle Loop")
                    .replace(side + " Double Pass - Trench Harvest", side + " Preload - Final Harvest");
            assertEquals(new JSONParser().parse(original), new JSONParser().parse(variant));
        }
    }

    @Test
    void mirrorsBothLegsExactlyAndConnectsAtAStoppedShootingPose() throws Exception {
        var outpost = PathPlannerAuto.getPathGroupFromAutoFile("Outpost Double Pass Bump Return");
        var depot = PathPlannerAuto.getPathGroupFromAutoFile("Depot Double Pass Bump Return");
        assertEquals(2, outpost.size());
        assertEquals(2, depot.size());
        for (int leg = 0; leg < 2; leg++) {
            var a = outpost.get(leg).getIdealTrajectory(config).orElseThrow();
            var b = depot.get(leg).getIdealTrajectory(config).orElseThrow();
            assertEquals(a.getTotalTimeSeconds(), b.getTotalTimeSeconds(), 1e-6);
            for (double t = 0; t <= a.getTotalTimeSeconds(); t += .02) {
                var left = a.sample(t);
                var right = b.sample(t);
                assertEquals(left.pose.getX(), right.pose.getX(), 1e-6);
                assertEquals(FieldConstants.fieldWidth - left.pose.getY(), right.pose.getY(), 1e-6);
                assertEquals(0, left.pose.getRotation().plus(right.pose.getRotation()).getRadians(), 1e-6);
                assertEquals(left.linearVelocity, right.linearVelocity, 1e-6);
            }
        }
        for (var paths : java.util.List.of(outpost, depot)) {
            var first = paths.get(0);
            assertEquals(FieldConstants.LinesVertical.starting,
                    first.getStartingHolonomicPose().orElseThrow().getX() + HALF_BUMPER, 1e-6,
                    "Front bumper must start on the line");
            var end = first.getWaypoints().get(first.getWaypoints().size() - 1).anchor();
            var nextStart = paths.get(1).getStartingHolonomicPose().orElseThrow();
            assertEquals(0, end.getDistance(nextStart.getTranslation()), 1e-6);
            // PathPlanner's adaptive sampling can stop a few millimeters before an anchor.
            assertEquals(0, first.getPoint(first.numPoints() - 1).position.getDistance(end), .01);
            assertEquals(first.getGoalEndState().rotation(), nextStart.getRotation());
            for (var path : paths) {
                assertEquals(0, path.getIdealStartingState().velocityMPS());
                assertEquals(0, path.getGoalEndState().velocityMPS());
            }
            var second = paths.get(1);
            var last = second.getWaypoints().get(second.getWaypoints().size() - 1).anchor();
            for (var shootingPose : new Pose2d[] {first.getStartingHolonomicPose().orElseThrow(), nextStart,
                    new Pose2d(last, second.getGoalEndState().rotation())}) {
                assertFalse(FieldTargeting.requiresHoodRetraction(shootingPose, new ChassisSpeeds()),
                        "Stationary shooting pose must permit the hood to extend");
                assertTrue(shootingPose.getX() + HALF_BUMPER <= FieldConstants.LinesVertical.starting + 1e-6);
            }
        }
    }

    @Test
    void crossesTrenchOutboundAndBumpInboundWithClearBumpers() throws Exception {
        for (String side : new String[] {"Outpost", "Depot"}) {
            var paths = PathPlannerAuto.getPathGroupFromAutoFile(side + " Double Pass Bump Return");
            boolean trench = false, bump = false, secondTrench = false, secondBump = false;
            for (int leg = 0; leg < 2; leg++) {
                var trajectory = paths.get(leg).getIdealTrajectory(config).orElseThrow();
                double previousX = trajectory.getInitialState().pose.getX();
                for (var sample : trajectory.getStates()) {
                    Pose2d pose = sample.pose;
                    double c = Math.abs(pose.getRotation().getCos()), s = Math.abs(pose.getRotation().getSin());
                    double extent = HALF_BUMPER * (c + s);
                    assertTrue(pose.getX() - extent >= 0 && pose.getX() + extent <= FieldConstants.fieldLength);
                    assertTrue(pose.getY() - extent >= 0 && pose.getY() + extent <= FieldConstants.fieldWidth);
                    double y = side.equals("Outpost") ? pose.getY() : FieldConstants.fieldWidth - pose.getY();
                    if (pose.getX() + extent >= FieldConstants.LinesVertical.starting
                            && pose.getX() - extent <= FieldConstants.Hub.farRightCorner.getX()) {
                        assertEquals(0, pose.getRotation().getRadians(), 1e-6);
                        if (pose.getX() >= previousX) {
                            assertTrue(y - extent > 0 && y + extent < FieldConstants.RightTrench.openingWidth,
                                    "Outbound trench clearance: " + pose);
                            if (leg == 0) trench = true;
                            else secondTrench = true;
                        } else {
                            assertTrue(y - extent > FieldConstants.LinesHorizontal.rightBumpEnd
                                    && y + extent < FieldConstants.Hub.nearRightCorner.getY(),
                                    "Return bump clearance: " + pose);
                            assertTrue(sample.linearVelocity <= 1.52, "Slow down before the bump");
                            if (leg == 0) bump = true;
                            else secondBump = true;
                        }
                    }
                    previousX = pose.getX();
                }
            }
            assertTrue(trench && bump && secondTrench && secondBump,
                    "Both passes must go out through the trench and return over the bump");
        }
    }

    @Test
    void everyCollectReturnAutoReturnsOverBumpAfterSecondPickup() throws Exception {
        for (String name : new String[] {"Outpost Preload Collect Return", "Depot Preload Collect Return",
                "Outpost Double Pass Bump Return", "Depot Double Pass Bump Return"}) {
            var paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
            var second = paths.get(1);
            var trajectory = second.getIdealTrajectory(config).orElseThrow();
            boolean crossed = false;
            double previousX = trajectory.getInitialState().pose.getX();
            for (var sample : trajectory.getStates()) {
                var pose = sample.pose;
                double extent = HALF_BUMPER * (Math.abs(pose.getRotation().getCos())
                        + Math.abs(pose.getRotation().getSin()));
                if (pose.getX() < previousX && pose.getX() + extent >= FieldConstants.LinesVertical.starting
                        && pose.getX() - extent <= FieldConstants.Hub.farRightCorner.getX()) {
                    double y = name.startsWith("Depot") ? FieldConstants.fieldWidth - pose.getY() : pose.getY();
                    assertTrue(y - extent > FieldConstants.LinesHorizontal.rightBumpEnd);
                    assertTrue(y + extent < FieldConstants.Hub.nearRightCorner.getY());
                    assertEquals(0, pose.getRotation().getRadians(), 1e-6);
                    assertTrue(sample.linearVelocity <= 1.52);
                    crossed = true;
                }
                previousX = pose.getX();
            }
            assertTrue(crossed, name + " needs a second bump return");
            assertTrue(trajectory.getEndState().pose.getX() + HALF_BUMPER < FieldConstants.LinesVertical.starting);
        }
    }

    @Test
    void transitUsesTheFullDriveModelAndHasNoCornersWithDiscontinuousTangents() throws Exception {
        for (String side : new String[] {"Outpost", "Depot"}) {
            for (var path : PathPlannerAuto.getPathGroupFromAutoFile(side + " Double Pass Bump Return")) {
                var global = path.getGlobalConstraints();
                assertEquals(config.moduleConfig.maxDriveVelocityMPS, global.maxVelocityMPS(), 1e-6);
                assertEquals(config.wheelFrictionForce * config.numModules / config.massKG,
                        global.maxAccelerationMPSSq(), 1e-6);
                var waypoints = path.getWaypoints();
                for (int i = 1; i < waypoints.size() - 1; i++) {
                    var w = waypoints.get(i);
                    var incoming = w.anchor().minus(w.prevControl());
                    var outgoing = w.nextControl().minus(w.anchor());
                    assertEquals(0, incoming.getAngle().minus(outgoing.getAngle()).getRadians(), 1e-6,
                            "An interior corner would force unnecessary slowing");
                }
                assertTrue(path.getIdealTrajectory(config).orElseThrow().getStates().stream()
                        .anyMatch(s -> s.linearVelocity > 3.1), "Use the available transit speed");
            }
        }
    }

    @Test
    void completesReturnShootingAndBothPickupsBeforeDeadline() throws Exception {
        for (String side : new String[] {"Outpost", "Depot"}) {
            var paths = PathPlannerAuto.getPathGroupFromAutoFile(side + " Double Pass Bump Return");
            double secondDeparture = 5 + paths.get(0).getIdealTrajectory(config).orElseThrow().getTotalTimeSeconds();
            assertTrue(secondDeparture < 20, "The return shot must complete before auto ends");
            for (int leg = 0; leg < 2; leg++) {
                intake = false;
                boolean collected = false;
                double pickupTime = Double.NaN;
                var trajectory = paths.get(leg).getIdealTrajectory(config).orElseThrow();
                var events = new EventScheduler();
                events.initialize(trajectory);
                try {
                    for (double t = 0; t <= trajectory.getTotalTimeSeconds() + .01; t += .005) {
                        events.execute(t);
                        if (intake) {
                            if (!collected) pickupTime = t;
                            collected = true;
                            assertTrue(trajectory.sample(t).linearVelocity <= 1.50,
                                    "Intake above pickup speed: " + side + " leg " + leg + " at " + t
                                            + " s: " + trajectory.sample(t).linearVelocity);
                        }
                    }
                } finally {
                    events.end();
                }
                assertTrue(collected);
                assertFalse(intake, "Intake must stop before each bump return and shot");
                if (leg == 1) {
                    double pickupStarts = secondDeparture + pickupTime;
                    double finalShotBegins = secondDeparture + trajectory.getTotalTimeSeconds();
                    System.out.printf("%s: second departure %.3f s, second pickup %.3f s, final shot %.3f s, available %.3f s%n",
                            side, secondDeparture, pickupStarts, finalShotBegins, 20-finalShotBegins);
                    assertTrue(pickupStarts < 20, "Second collection must start before the 20-second deadline");
                    assertTrue(finalShotBegins < 17.5,
                            "Return from the full second sweep with at least 2.5 s for a shooting attempt");
                }
            }
        }
    }
}
