package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCompensation {
    /** Angle is field-relative; tracking rate is robot-relative, CCW positive. */
    public record AdjustedShot(double rpm, double hoodAngleDeg, double turretAngleDeg,
                               double turretFF_deg_s, double rangeFF_m_s) {}

    /**
     * Fixed-flight-time variant for a launcher at the robot center. Speeds are
     * field-relative. Use MovingShotSolver for the actual offset turret and the
     * range-dependent empirical flight time. The returned rate is robot-relative.
     */
    public static AdjustedShot compensateForMovement(
            Pose2d robotPose, ChassisSpeeds fieldRelVel, Pose2d targetPose, double nominalShotTimeS) {
        if (!Double.isFinite(nominalShotTimeS) || nominalShotTimeS <= 0.0) {
            throw new IllegalArgumentException("Shot flight time must be finite and positive");
        }
        Translation2d velocity = new Translation2d(
                fieldRelVel.vxMetersPerSecond, fieldRelVel.vyMetersPerSecond);
        Translation2d aim = targetPose.getTranslation().minus(robotPose.getTranslation())
                .minus(velocity.times(nominalShotTimeS));
        // Preserve the full vector, even when forward velocity exceeds range/TOF.
        double range = aim.getNorm();
        double fieldRate = range < 1e-6 ? 0.0
                : (aim.getY() * velocity.getX() - aim.getX() * velocity.getY()) / (range * range);
        double rangeRate = range < 1e-6 ? 0.0
                : -(aim.getX() * velocity.getX() + aim.getY() * velocity.getY()) / range;
        return new AdjustedShot(ShotData.distanceToRPM.get(range),
                ShotData.distanceToHoodAngle.get(range), aim.getAngle().getDegrees(),
                Math.toDegrees(fieldRate - fieldRelVel.omegaRadiansPerSecond), rangeRate);
    }
}
