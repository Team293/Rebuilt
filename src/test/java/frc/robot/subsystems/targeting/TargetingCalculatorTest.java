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
    void consumersShareOneImmutableSolutionUntilTheNextUpdate() {
        var calculator = new TargetingCalculator();
        var target = new Translation2d(8, 5);
        calculator.update(new Pose2d(2, 2, Rotation2d.kZero), new ChassisSpeeds(), target);
        var initial = calculator.getSolution();
        calculator.update(new Pose2d(3, 1, Rotation2d.kZero), new ChassisSpeeds(), target);
        var turretSolution = calculator.getSolution();
        for (int i = 0; i < 100; i++) {
            var shooterSolution = calculator.getSolution();
            assertSame(turretSolution, shooterSolution);
            assertSame(turretSolution.shot(), shooterSolution.shot());
            assertEquals(shooterSolution.toGoal().getNorm(), shooterSolution.rangeMeters(), EPSILON);
            assertEquals(shooterSolution.toGoal().getAngle(), turretSolution.fieldRelativeDirection());
        }
        var expected = target.minus(new Translation2d(3, 1).plus(Turret.TURRET_OFFSET_FROM_CENTER));
        assertTranslation(expected, turretSolution.toGoal());
        assertNotEquals(initial.toGoal(), turretSolution.toGoal());
        calculator.update(new Pose2d(4, 1, Rotation2d.k180deg), new ChassisSpeeds(), target);
        assertNotSame(turretSolution, calculator.getSolution());
        // Previously published solutions remain unchanged after subsequent updates.
        assertTranslation(expected, turretSolution.toGoal());
    }

    @Test
    void snapshotControlsPivotVelocityFlightTimeAndFeedforwardTogether() {
        var calculator = new TargetingCalculator();
        var pose = new Pose2d(2, 3, Rotation2d.kCCW_90deg);
        var speeds = new ChassisSpeeds(2, 1, 0.7);
        var target = new Translation2d(8, 6);
        calculator.update(pose, speeds, target);
        var solution = calculator.getSolution();
        // At 90 degrees, robot (vx=2, vy=1) becomes field (vx=-1, vy=2).
        // The pivot offset becomes (0.2, -0.3), with omega x r = (0.21, 0.14).
        var pivot = new Translation2d(2.2, 2.7);
        var velocity = new Translation2d(-0.79, 2.14);
        assertTranslation(pivot, solution.turretPivot());
        assertTranslation(velocity, solution.turretPivotVelocity());
        assertEquals(target.getDistance(pivot), solution.staticDistanceMeters(), EPSILON);
        assertTranslation(target.minus(pivot).minus(velocity.times(solution.timeOfFlightSeconds())),
                solution.toGoal());
        assertEquals(ShotData.distanceToTOFConstant.get(solution.rangeMeters()),
                solution.timeOfFlightSeconds(), EPSILON);
        assertEquals(solution.fieldRelativeDirection().getDegrees(), solution.shot().turretAngleDeg(), EPSILON);
        assertEquals(ShotData.distanceToRPM.get(solution.rangeMeters()), solution.shot().rpm(), EPSILON);
        speeds.vxMetersPerSecond = 100;
        speeds.omegaRadiansPerSecond = -100;
        assertTranslation(velocity, calculator.getSolution().turretPivotVelocity());
        assertSame(solution, calculator.getSolution());
    }

    @Test
    void targetChangesIncludingAllianceChangesCannotCarryHistory() {
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
            calculator.update(pose, speeds, target);
            var fresh = new TargetingCalculator();
            fresh.update(pose, speeds, target);
            assertEquals(fresh.getSolution(), calculator.getSolution());
            calculator.update(new Pose2d(5, 2, Rotation2d.k180deg), speeds, target);
        }
    }

    @Test
    void odometryResetsCannotCarryHistoryEvenWhenTheTargetDoesNotChange() {
        var calculator = new TargetingCalculator();
        var target = new Translation2d(8, 5);
        var speeds = new ChassisSpeeds(0.5, 1.5, -0.4);
        calculator.update(new Pose2d(1, 1, Rotation2d.kZero), speeds, target);
        for (var resetPose : new Pose2d[] {
                new Pose2d(1, 1, Rotation2d.kCCW_90deg),
                new Pose2d(4, 2, Rotation2d.kCCW_90deg),
                new Pose2d(5, 2, Rotation2d.k180deg)}) {
            calculator.update(resetPose, speeds, target);
            var fresh = new TargetingCalculator();
            fresh.update(resetPose, speeds, target);
            assertEquals(fresh.getSolution(), calculator.getSolution());
        }
    }

    private void assertTranslation(Translation2d expected, Translation2d actual) {
        assertEquals(expected.getX(), actual.getX(), EPSILON);
        assertEquals(expected.getY(), actual.getY(), EPSILON);
    }
}
