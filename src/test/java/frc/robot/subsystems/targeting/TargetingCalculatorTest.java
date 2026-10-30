package frc.robot.subsystems.targeting;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.FieldConstants;
import frc.robot.subsystems.turret.Turret;
import org.junit.jupiter.api.Test;

class TargetingCalculatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    void consumersShareOneSolutionAndReadsDoNotAdvanceTheTwentyMillisecondFilter() {
        var calculator = new TargetingCalculator();
        var target = new Translation2d(8, 5);
        var initialPose = new Pose2d(2, 2, Rotation2d.kZero);
        calculator.update(initialPose, new ChassisSpeeds(), target, 0);
        var initial = calculator.getSolution();
        var nextPose = new Pose2d(3, 1, Rotation2d.kZero);
        calculator.update(nextPose, new ChassisSpeeds(), target, 0);

        var turretSolution = calculator.getSolution();
        for (int i = 0; i < 100; i++) {
            var shooterSolution = calculator.getSolution();
            assertSame(turretSolution, shooterSolution);
            assertEquals(shooterSolution.toGoal().getNorm(), shooterSolution.rangeMeters(), EPSILON);
            assertEquals(shooterSolution.toGoal().getAngle(), turretSolution.fieldRelativeDirection());
        }

        // One 20 ms sample with a 40 ms time constant, regardless of consumer count.
        double retained = Math.exp(-0.02 / 0.04);
        Translation2d raw = target.minus(nextPose.getTranslation().plus(Turret.TURRET_OFFSET_FROM_CENTER));
        Translation2d expected = initial.toGoal().times(retained).plus(raw.times(1 - retained));
        assertTranslation(expected, turretSolution.toGoal());

        calculator.update(nextPose, new ChassisSpeeds(), target, 0);
        assertNotSame(turretSolution, calculator.getSolution());
        assertTranslation(expected.times(retained).plus(raw.times(1 - retained)),
                calculator.getSolution().toGoal());
        // Previously published solutions remain unchanged after subsequent updates.
        assertTranslation(expected, turretSolution.toGoal());
    }

    @Test
    void initialSolutionUsesTheSnapshotForTranslationRotationAndTurretOffset() {
        var calculator = new TargetingCalculator();
        var pose = new Pose2d(2, 3, Rotation2d.kCCW_90deg);
        var speeds = new ChassisSpeeds(2, 1, 0.7);
        var target = new Translation2d(8, 6);
        calculator.update(pose, speeds, target, 0);
        var solution = calculator.getSolution();
        double distance = target.getDistance(pose.getTranslation().plus(
                Turret.TURRET_OFFSET_FROM_CENTER.rotateBy(pose.getRotation())));
        assertEquals(distance, solution.staticDistanceMeters(), EPSILON);
        double tof = ShotData.distanceToTOFConstant.get(distance);
        assertEquals(tof, solution.timeOfFlightSeconds(), EPSILON);
        // At 90 degrees, robot (vx=2, vy=1) becomes field (vx=-1, vy=2).
        var predictedPosition = new Translation2d(2 - tof, 3 + 2 * tof);
        var predictedHeading = Rotation2d.fromRadians(Math.PI / 2 + 0.7 * tof);
        var pivot = predictedPosition.plus(Turret.TURRET_OFFSET_FROM_CENTER.rotateBy(predictedHeading));
        assertTranslation(predictedPosition, solution.predictedRobotPose().getTranslation());
        assertEquals(0, predictedHeading.minus(solution.predictedRobotPose().getRotation()).getRadians(), EPSILON);
        assertTranslation(pivot, solution.predictedTurretPivot().getTranslation());
        assertTranslation(target.minus(pivot), solution.toGoal());
        speeds.vxMetersPerSecond = 100;
        speeds.omegaRadiansPerSecond = -100;
        assertTranslation(target.minus(pivot), calculator.getSolution().toGoal());
    }

    @Test
    void targetChangesIncludingAllianceChangesDiscardAllFilterHistory() {
        var calculator = new TargetingCalculator();
        var pose = new Pose2d(3, 4, Rotation2d.fromDegrees(25));
        var speeds = new ChassisSpeeds(1, -0.5, 0.3);
        Translation2d[] targets = {
            FieldConstants.Hub.innerCenterPoint.toTranslation2d(),
            new Translation2d(2, 2.5),
            new Translation2d(2, FieldConstants.fieldWidth - 2.5),
            FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
        };
        for (var target : targets) {
            calculator.update(pose, speeds, target, 0);
            var fresh = new TargetingCalculator();
            fresh.update(pose, speeds, target, 0);
            assertSolution(fresh.getSolution(), calculator.getSolution());
            // Accumulate different history before changing targets again.
            for (int i = 0; i < 8; i++) {
                calculator.update(new Pose2d(5, 2, Rotation2d.k180deg), speeds, target, 0);
            }
        }
    }

    @Test
    void odometryResetsDiscardHistoryEvenWhenTheTargetDoesNotChange() {
        var calculator = new TargetingCalculator();
        var target = new Translation2d(8, 5);
        var speeds = new ChassisSpeeds(0.5, 1.5, -0.4);
        calculator.update(new Pose2d(1, 1, Rotation2d.kZero), speeds, target, 0);
        for (int revision = 1; revision <= 3; revision++) {
            var resetPose = new Pose2d(2 + revision, 2, Rotation2d.fromDegrees(60 * revision));
            calculator.update(resetPose, speeds, target, revision);
            var fresh = new TargetingCalculator();
            fresh.update(resetPose, speeds, target, revision);
            assertSolution(fresh.getSolution(), calculator.getSolution());
        }
    }

    private void assertSolution(TargetingSolution expected, TargetingSolution actual) {
        assertTranslation(expected.toGoal(), actual.toGoal());
        assertEquals(expected.staticDistanceMeters(), actual.staticDistanceMeters(), EPSILON);
        assertEquals(expected.timeOfFlightSeconds(), actual.timeOfFlightSeconds(), EPSILON);
        assertEquals(expected.predictedRobotPose(), actual.predictedRobotPose());
        assertEquals(expected.predictedTurretPivot(), actual.predictedTurretPivot());
    }

    private void assertTranslation(Translation2d expected, Translation2d actual) {
        assertEquals(expected.getX(), actual.getX(), EPSILON);
        assertEquals(expected.getY(), actual.getY(), EPSILON);
    }
}
