package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Release-now model: the projectile inherits the pivot's instantaneous field velocity. */
public final class MovingShotSolver {
    private MovingShotSolver() {}

    public record Solution(Translation2d toGoal, Translation2d pivotPosition,
                           Translation2d pivotVelocity, double timeOfFlightS,
                           ShotCompensation.AdjustedShot shot) {}

    private record Geometry(Translation2d pivot, Translation2d velocity,
                            Translation2d aim, double tof) {}

    private static final double RATE_STEP_S = 1e-4;

    /**
     * Speeds are FIELD-relative at the robot center; pivotOffset is ROBOT-relative.
     * q = target - pivot - (v_center + omega x r) * TOF(|q|).
     * Once released, the projectile does not follow the turret's circular path.
     * Tracking rates assume constant field-relative center velocity and yaw rate.
     * This models release at the pivot; muzzle offset, release latency and drag
     * require physical measurements, as does the empirical flight-time table.
     */
    public static Solution solve(Pose2d pose, ChassisSpeeds fieldSpeeds,
                                  Translation2d target, Translation2d pivotOffset) {
        Geometry now = geometry(pose, fieldSpeeds, target, pivotOffset, 0.0);
        // Differentiate the full solution, including rotating pivot velocity and
        // changing flight time. Angle modulus prevents spikes at +/-180 degrees.
        Geometry before = geometry(pose, fieldSpeeds, target, pivotOffset, -RATE_STEP_S);
        Geometry after = geometry(pose, fieldSpeeds, target, pivotOffset, RATE_STEP_S);
        double fieldRate = now.aim().getNorm() < 1e-6 ? 0.0 : MathUtil.angleModulus(
                after.aim().getAngle().getRadians() - before.aim().getAngle().getRadians())
                / (2.0 * RATE_STEP_S);
        double rangeRate = (after.aim().getNorm() - before.aim().getNorm()) / (2.0 * RATE_STEP_S);
        double range = now.aim().getNorm();
        var shot = new ShotCompensation.AdjustedShot(
                ShotData.distanceToRPM.get(range), ShotData.distanceToHoodAngle.get(range),
                now.aim().getAngle().getDegrees(),
                Math.toDegrees(fieldRate - fieldSpeeds.omegaRadiansPerSecond), rangeRate);
        return new Solution(now.aim(), now.pivot(), now.velocity(), now.tof(), shot);
    }

    private static Geometry geometry(Pose2d pose, ChassisSpeeds speeds,
                                       Translation2d target, Translation2d offset, double dt) {
        Translation2d center = pose.getTranslation().plus(new Translation2d(
                speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt));
        Translation2d r = offset.rotateBy(pose.getRotation().plus(
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
        Translation2d pivot = center.plus(r);
        Translation2d velocity = new Translation2d(
                speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
                speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
        Translation2d displacement = target.minus(pivot);
        // The positive, increasing TOF table clamps at both endpoints. Bisect
        // t - TOF(|displacement - velocity*t|); fixed-point iteration can oscillate.
        double low = ShotData.distanceToTOFConstant.get(0.0);
        double high = ShotData.distanceToTOFConstant.get(Double.MAX_VALUE);
        for (int i = 0; i < 40; i++) {
            double mid = (low + high) * 0.5;
            double range = displacement.minus(velocity.times(mid)).getNorm();
            if (mid < ShotData.distanceToTOFConstant.get(range)) {
                low = mid;
            } else {
                high = mid;
            }
        }
        double tof = (low + high) * 0.5;
        return new Geometry(pivot, velocity, displacement.minus(velocity.times(tof)), tof);
    }
}
