package frc.robot.subsystems.targeting;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.FieldConstants;
import frc.robot.subsystems.targeting.Targeting.Target;
import org.junit.jupiter.api.Test;

class FieldTargetingTest {
    private Pose2d pose(double x, double y) {
        return new Pose2d(x, y, Rotation2d.kZero);
    }

    @Test
    void choosesHubAtHomeAndNearestShuttleSideAwayForBothAlliances() {
        double center = FieldConstants.LinesHorizontal.center;
        for (boolean red : new boolean[] {false, true}) {
            for (double y : new double[] {1, center, FieldConstants.fieldWidth - 1}) {
                assertEquals(Target.HUB, FieldTargeting.selectTarget(alliancePose(2, y, red), red, Target.HUB));
            }
            assertEquals(Target.SHUTTLE_RIGHT,
                    FieldTargeting.selectTarget(alliancePose(8, 1, red), red, Target.HUB));
            assertEquals(Target.SHUTTLE_LEFT,
                    FieldTargeting.selectTarget(alliancePose(8, FieldConstants.fieldWidth - 1, red), red, Target.HUB));
        }
    }

    private Pose2d alliancePose(double x, double y, boolean red) {
        return red ? pose(FieldConstants.fieldLength - x, FieldConstants.fieldWidth - y) : pose(x, y);
    }

    @Test
    void keepsTargetsStableUntilCrossingTheHysteresisBand() {
        double boundary = FieldConstants.LinesVertical.allianceZone;
        double center = FieldConstants.LinesHorizontal.center;
        double h = FieldTargeting.TARGET_HYSTERESIS_METERS;
        for (boolean red : new boolean[] {false, true}) {
            assertEquals(Target.HUB,
                    FieldTargeting.selectTarget(alliancePose(boundary + h / 2, 1, red), red, Target.HUB));
            assertEquals(Target.SHUTTLE_RIGHT,
                    FieldTargeting.selectTarget(alliancePose(boundary + 2 * h, 1, red), red, Target.HUB));
            assertEquals(Target.SHUTTLE_RIGHT,
                    FieldTargeting.selectTarget(alliancePose(boundary - h / 2, 1, red), red, Target.SHUTTLE_RIGHT));
            assertEquals(Target.HUB,
                    FieldTargeting.selectTarget(alliancePose(boundary - 2 * h, 1, red), red, Target.SHUTTLE_RIGHT));
            assertEquals(Target.SHUTTLE_LEFT,
                    FieldTargeting.selectTarget(alliancePose(8, center - h / 2, red), red, Target.SHUTTLE_LEFT));
            assertEquals(Target.SHUTTLE_RIGHT,
                    FieldTargeting.selectTarget(alliancePose(8, center - 2 * h, red), red, Target.SHUTTLE_LEFT));
            assertEquals(Target.SHUTTLE_RIGHT,
                    FieldTargeting.selectTarget(alliancePose(8, center + h / 2, red), red, Target.SHUTTLE_RIGHT));
            assertEquals(Target.SHUTTLE_LEFT,
                    FieldTargeting.selectTarget(alliancePose(8, center + 2 * h, red), red, Target.SHUTTLE_RIGHT));
        }
    }

    @Test
    void retractsAtAllFourTrenchesEvenWhileStoppedOrFacingSideways() {
        for (double x : new double[] {FieldConstants.LinesVertical.hubCenter, FieldConstants.LinesVertical.oppHubCenter}) {
            for (double y : new double[] {0.6, FieldConstants.fieldWidth - 0.6}) {
                for (double heading : new double[] {0, 45, 90, 180}) {
                    assertTrue(FieldTargeting.requiresHoodRetraction(
                            new Pose2d(x, y, Rotation2d.fromDegrees(heading)), new ChassisSpeeds()));
                }
            }
        }
    }

    @Test
    void anticipatesApproachFromEitherDirectionAndConvertsRobotRelativeVelocity() {
        double x = FieldConstants.LinesVertical.hubCenter;
        for (double side : new double[] {-1, 1}) {
            Pose2d approach = new Pose2d(x + side * 2, 0.6, Rotation2d.fromDegrees(90));
            assertFalse(FieldTargeting.requiresHoodRetraction(approach, new ChassisSpeeds()));
            // At 90 degrees robot +Y points toward field -X.
            assertTrue(FieldTargeting.requiresHoodRetraction(approach, new ChassisSpeeds(0, side * 3, 0)));
            assertFalse(FieldTargeting.requiresHoodRetraction(approach, new ChassisSpeeds(0, -side * 3, 0)));
        }
    }

    @Test
    void detectsSweptCrossingEvenWhenBothEndpointsAreOutside() {
        double x = FieldConstants.LinesVertical.hubCenter;
        assertTrue(FieldTargeting.requiresHoodRetraction(pose(x - 1.5, 0.6), new ChassisSpeeds(6, 0, 0)));
    }

    @Test
    void keepsHoodRetractedUntilTheEntireRobotClearsTheTrench() {
        double edge = FieldConstants.LinesVertical.hubCenter + FieldConstants.RightTrench.depth / 2;
        double padding = FieldTargeting.ROBOT_RADIUS_METERS + FieldTargeting.TRENCH_MARGIN_METERS;
        assertTrue(FieldTargeting.requiresHoodRetraction(pose(edge + padding - 0.01, 0.6), new ChassisSpeeds()));
        assertFalse(FieldTargeting.requiresHoodRetraction(pose(edge + padding + 0.01, 0.6), new ChassisSpeeds()));
    }

    @Test
    void permitsNormalHoodMotionAwayFromTrenchesIncludingTheBumpsAndMidfield() {
        for (double x : new double[] {2, FieldConstants.LinesVertical.hubCenter,
                FieldConstants.LinesVertical.center, FieldConstants.LinesVertical.oppHubCenter,
                FieldConstants.fieldLength - 2}) {
            assertFalse(FieldTargeting.requiresHoodRetraction(
                    pose(x, FieldConstants.LinesHorizontal.center), new ChassisSpeeds(3, 0, 0)));
        }
        assertFalse(FieldTargeting.requiresHoodRetraction(pose(2, 0.6), new ChassisSpeeds()));
    }
}
