package frc.robot.subsystems.turret.calc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCompensation {

    public record AdjustedShot(double rpm, double hoodAngleDeg, double turretAngleDeg,
                               double turretFF_deg_s, double rangeFF_m_s) {}

    public static AdjustedShot compensateForMovement(
            Pose2d robotPose,
            ChassisSpeeds fieldRelVel,
            Pose2d targetPose,
            double nominalShotTimeS
    ) {

        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        double rangeM = toTarget.getNorm();

        Rotation2d robotToGoalRot = toTarget.getAngle();

        double uncompensatedTurretDeg = robotToGoalRot.getDegrees();

        Translation2d velTrans = new Translation2d(fieldRelVel.vxMetersPerSecond, fieldRelVel.vyMetersPerSecond);
        Translation2d velRelative = velTrans.rotateBy(robotToGoalRot.unaryMinus());

        double radialM_s   = velRelative.getX();
        double tangentialM_s = velRelative.getY();

        double angularDeg_s = Math.toDegrees(fieldRelVel.omegaRadiansPerSecond);

        double effectiveRangeM;
        double effectiveTurretDeg = uncompensatedTurretDeg;

        double shotHorizontalSpeed = rangeM / nominalShotTimeS - radialM_s;
        if (shotHorizontalSpeed < 0) shotHorizontalSpeed = 0;

        double leadAdjustmentDeg = Math.toDegrees(
                Math.atan2(-tangentialM_s, shotHorizontalSpeed)
        );

        effectiveTurretDeg += leadAdjustmentDeg;

        effectiveRangeM = nominalShotTimeS *
                Math.hypot(tangentialM_s, shotHorizontalSpeed);

        double baseRPM = ShotData.distanceToRPM.get(effectiveRangeM);
        double hoodDeg = ShotData.distanceToHoodAngle.get(effectiveRangeM);

        double turretFF_deg_s = -(angularDeg_s + Math.toDegrees(tangentialM_s / rangeM));
        double rangeFF_m_s    = -radialM_s;

        // double adjustedRPM = baseRPM + rangeFF_m_s * someGain_RPM_per_m_s;  // tune gain

        return new AdjustedShot(baseRPM, hoodDeg, effectiveTurretDeg,
                turretFF_deg_s, rangeFF_m_s);
    }

    private static double calculateVelocityCompensation(
            ChassisSpeeds velocity,
            Translation2d directionToTarget) {

        double robotSpeedTowardTarget =
                velocity.vxMetersPerSecond * Math.cos(directionToTarget.getAngle().getRadians()) +
                        velocity.vyMetersPerSecond * Math.sin(directionToTarget.getAngle().getRadians());

        // tune this constant empirically
        return robotSpeedTowardTarget * 100.0;
    }

    private static double rpmToVelocity(double rpm) {
        // 6 inch wheel: 6 * 0.0254 = 0.1524 m diameter
        return rpm * (Math.PI * 0.1524) / 60.0;
    }
}