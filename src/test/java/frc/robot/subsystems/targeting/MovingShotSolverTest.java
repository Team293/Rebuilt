package frc.robot.subsystems.targeting;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class MovingShotSolverTest {
    private static final Translation2d OFFSET = new Translation2d(-0.3, -0.2);

    @Test
    void fixedTimeVariantPreservesReverseAimAndUsesCompensatedRate() {
        var shot = ShotCompensation.compensateForMovement(new Pose2d(),
                new ChassisSpeeds(5, 1, 0.5), new Pose2d(2, 0, new Rotation2d()), 1.0);
        Translation2d aim = new Translation2d(-3, -1);
        assertEquals(aim.getAngle().getDegrees(), shot.turretAngleDeg(), 1e-10);
        assertEquals(Math.toDegrees(-0.2 - 0.5), shot.turretFF_deg_s(), 1e-10);
        assertThrows(IllegalArgumentException.class, () -> ShotCompensation.compensateForMovement(
                new Pose2d(), new ChassisSpeeds(), new Pose2d(), 0));
        var coincident = ShotCompensation.compensateForMovement(
                new Pose2d(), new ChassisSpeeds(), new Pose2d(), 1);
        assertTrue(Double.isFinite(coincident.turretFF_deg_s()));
    }

    @Test
    void stationaryShotUsesActualPivotAndEmpiricalTables() {
        Pose2d pose = new Pose2d(2, 3, Rotation2d.fromDegrees(90));
        Translation2d target = new Translation2d(6, 4);
        var solution = MovingShotSolver.solve(pose, new ChassisSpeeds(), target, OFFSET);
        Translation2d expected = target.minus(new Translation2d(2.2, 2.7));
        assertVector(expected, solution.toGoal(), 1e-12);
        assertEquals(ShotData.distanceToTOFConstant.get(expected.getNorm()), solution.timeOfFlightS(), 1e-10);
        assertEquals(ShotData.distanceToRPM.get(expected.getNorm()), solution.shot().rpm(), 1e-10);
        assertEquals(ShotData.distanceToHoodAngle.get(expected.getNorm()), solution.shot().hoodAngleDeg(), 1e-10);
        assertEquals(0, solution.shot().turretFF_deg_s(), 1e-8);
    }

    @Test
    void rotationAddsInstantaneousTangentialVelocityWithCorrectSign() {
        for (double omega : new double[] {-6, -2, 2, 6}) {
            var solution = MovingShotSolver.solve(new Pose2d(), new ChassisSpeeds(0, 0, omega),
                    new Translation2d(4, 0), OFFSET);
            assertVector(new Translation2d(0.2 * omega, -0.3 * omega), solution.pivotVelocity(), 1e-12);
            assertVector(new Translation2d(4.3 - 0.2 * omega * solution.timeOfFlightS(),
                    0.2 + 0.3 * omega * solution.timeOfFlightS()), solution.toGoal(), 1e-10);
        }
    }

    @Test
    void centeredTurretCounterRotatesWithoutAddingBallisticYawLead() {
        for (double omega : new double[] {-8, -1, 0, 1, 8}) {
            var solution = MovingShotSolver.solve(new Pose2d(), new ChassisSpeeds(0, 0, omega),
                    new Translation2d(-4, 0), new Translation2d());
            assertVector(new Translation2d(-4, 0), solution.toGoal(), 1e-12);
            assertEquals(-Math.toDegrees(omega), solution.shot().turretFF_deg_s(), 1e-8);
        }
    }

    @Test
    void simulatedProjectileHitsTargetAcrossCombinedTranslationAndRotation() {
        Random random = new Random(293);
        for (int i = 0; i < 1000; i++) {
            Pose2d pose = new Pose2d(random.nextDouble() * 8, random.nextDouble() * 8,
                    Rotation2d.fromRadians(random.nextDouble() * 2 * Math.PI));
            ChassisSpeeds speeds = new ChassisSpeeds(random.nextDouble() * 8 - 4,
                    random.nextDouble() * 8 - 4, random.nextDouble() * 12 - 6);
            Translation2d target = pose.getTranslation().plus(new Translation2d(
                    random.nextDouble() * 12 - 6, random.nextDouble() * 12 - 6));
            var solution = MovingShotSolver.solve(pose, speeds, target, OFFSET);
            // Independently differentiate the physical pivot trajectory to obtain
            // inherited velocity, then propagate the ball freely after release.
            double dt = 1e-6;
            Translation2d pivot = physicalPivot(pose, speeds, 0);
            Translation2d inherited = physicalPivot(pose, speeds, dt)
                    .minus(physicalPivot(pose, speeds, -dt)).div(2 * dt);
            double time = solution.timeOfFlightS();
            Translation2d muzzleVelocity = new Translation2d(
                    solution.toGoal().getNorm() / time,
                    Rotation2d.fromDegrees(solution.shot().turretAngleDeg()));
            Translation2d impact = pivot.plus(muzzleVelocity.plus(inherited).times(time));
            assertVector(target, impact, 1e-7);
            assertEquals(ShotData.distanceToTOFConstant.get(solution.toGoal().getNorm()), time, 1e-9);
            assertTrue(Double.isFinite(solution.shot().turretFF_deg_s()));
        }
    }

    @Test
    void solutionIsInvariantUnderFieldRotation() {
        Pose2d pose = new Pose2d(2, 1, Rotation2d.fromDegrees(57));
        Translation2d target = new Translation2d(6, 4);
        ChassisSpeeds speeds = new ChassisSpeeds(1.2, -0.7, 2.1);
        var original = MovingShotSolver.solve(pose, speeds, target, OFFSET);
        Rotation2d rotation = Rotation2d.fromDegrees(133);
        Translation2d rotatedV = new Translation2d(1.2, -0.7).rotateBy(rotation);
        var rotated = MovingShotSolver.solve(
                new Pose2d(pose.getTranslation().rotateBy(rotation), pose.getRotation().plus(rotation)),
                new ChassisSpeeds(rotatedV.getX(), rotatedV.getY(), 2.1), target.rotateBy(rotation), OFFSET);
        assertVector(original.toGoal().rotateBy(rotation), rotated.toGoal(), 1e-9);
        assertEquals(original.shot().turretFF_deg_s(), rotated.shot().turretFF_deg_s(), 1e-6);
    }

    @Test
    void feedforwardMatchesRobotRelativeAimTrajectoryIncludingWrap() {
        Pose2d pose = new Pose2d(2, 1, Rotation2d.fromDegrees(32));
        for (Translation2d target : new Translation2d[] {
                new Translation2d(6, 4), new Translation2d(-4, 1)}) {
            ChassisSpeeds speeds = new ChassisSpeeds(1.2, -0.7, 2.1);
            var now = MovingShotSolver.solve(pose, speeds, target, OFFSET);
            double dt = 1e-3;
            var before = MovingShotSolver.solve(advance(pose, speeds, -dt), speeds, target, OFFSET);
            var after = MovingShotSolver.solve(advance(pose, speeds, dt), speeds, target, OFFSET);
            double robotAngleBefore = before.toGoal().getAngle().getRadians()
                    - advance(pose, speeds, -dt).getRotation().getRadians();
            double robotAngleAfter = after.toGoal().getAngle().getRadians()
                    - advance(pose, speeds, dt).getRotation().getRadians();
            double expected = Math.toDegrees(MathUtil.angleModulus(robotAngleAfter - robotAngleBefore) / (2 * dt));
            assertEquals(expected, now.shot().turretFF_deg_s(), 0.002);
            assertEquals((after.toGoal().getNorm() - before.toGoal().getNorm()) / (2 * dt),
                    now.shot().rangeFF_m_s(), 0.0001);
        }
    }

    @Test
    void trackingRateRemainsContinuousAtFieldAngleWrap() {
        // TOF(4 m) = 1.4 s, so the compensated vector lies exactly on -X.
        // Lateral motion crosses +/-180 degrees; expected field bearing rate is 1/4 rad/s.
        var solution = MovingShotSolver.solve(new Pose2d(), new ChassisSpeeds(0, 1, 2),
                new Translation2d(-4, 1.4), new Translation2d());
        assertVector(new Translation2d(-4, 0), solution.toGoal(), 1e-10);
        assertEquals(Math.toDegrees(0.25 - 2), solution.shot().turretFF_deg_s(), 0.001);
    }

    @Test
    void coincidentTargetRemainsFinite() {
        var solution = MovingShotSolver.solve(new Pose2d(), new ChassisSpeeds(), OFFSET, OFFSET);
        assertEquals(0, solution.toGoal().getNorm());
        assertTrue(Double.isFinite(solution.shot().turretFF_deg_s()));
        assertTrue(Double.isFinite(solution.shot().rangeFF_m_s()));
    }

    private static Pose2d advance(Pose2d pose, ChassisSpeeds speeds, double dt) {
        return new Pose2d(pose.getX() + speeds.vxMetersPerSecond * dt,
                pose.getY() + speeds.vyMetersPerSecond * dt,
                pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
    }

    private static Translation2d physicalPivot(Pose2d pose, ChassisSpeeds speeds, double dt) {
        Pose2d advanced = advance(pose, speeds, dt);
        return advanced.getTranslation().plus(OFFSET.rotateBy(advanced.getRotation()));
    }

    private static void assertVector(Translation2d expected, Translation2d actual, double tolerance) {
        assertEquals(expected.getX(), actual.getX(), tolerance);
        assertEquals(expected.getY(), actual.getY(), tolerance);
    }
}
