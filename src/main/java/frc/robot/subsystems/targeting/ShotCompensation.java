package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCompensation {

    public record AdjustedShot(double rpm, double hoodAngleDeg, double turretAngleDeg,
                               double turretFF_deg_s, double rangeFF_m_s) {}

    /**
     * Calculate adjustments to shot parameters to compensate for robot movement during the shot, using a simple linear prediction of target motion based on current velocity.
     * @param robotPose current robot pose on the field
     * @param fieldRelVel current robot velocity in field-relative coordinates (x is forward, y is left, omega is CCW positive), used to predict where the target will be when the shot arrives
     * @param targetPose current target pose on the field, used to calculate initial aiming angle and distance for compensation calculations
     * @param nominalShotTimeS estimated time from shot release to target impact at the nominal shot speed, used to predict where the target will be when the shot arrives. This should be based on empirical measurements of the actual shot time for the given shot parameters, and is critical for accurate compensation.
     * @return adjusted shot parameters including feedforward terms for turret angle and shot speed to compensate for robot movement, as well as the effective turret angle to aim at after compensation. The caller should apply the feedforward terms to the turret control and shot speed commands to achieve the desired compensation.
     */
    public static AdjustedShot compensateForMovement(
            Pose2d robotPose,
            ChassisSpeeds fieldRelVel,
            Pose2d targetPose,
            double nominalShotTimeS
    ) {

        // vector from robot to target in field coords
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        // straight line distance to target (meters)
        double rangeM = toTarget.getNorm();

        // angle from robot to goal, in field/world space
        Rotation2d robotToGoalRot = toTarget.getAngle();

        // raw turret angle pointing directly at the goal (no movement compensation)
        double uncompensatedTurretDeg = robotToGoalRot.getDegrees();

        // convert chassis velocities into a 2d translation for easier math
        Translation2d velTrans = new Translation2d(fieldRelVel.vxMetersPerSecond, fieldRelVel.vyMetersPerSecond);
        // rotate the velocity vector into the frame that points toward the goal
        // this makes the x component be the radial (toward/away) speed and y be tangential (sideways)
        Translation2d velRelative = velTrans.rotateBy(robotToGoalRot.unaryMinus());

        // radial: how fast robot is moving toward/away from the target (m/s)
        double radialM_s   = velRelative.getX();
        // tangential: sideways speed relative to the target (m/s)
        double tangentialM_s = velRelative.getY();

        // convert chassis angular velocity to degrees per second so it's easier to reason about
        double angularDeg_s = Math.toDegrees(fieldRelVel.omegaRadiansPerSecond);

        double effectiveRangeM;
        // start with the angle you'd aim if the robot were stationary
        double effectiveTurretDeg = uncompensatedTurretDeg;

        // estimate horizontal shot speed needed to reach target in nominalShotTimeS, then remove radial robot motion
        double shotHorizontalSpeed = rangeM / nominalShotTimeS - radialM_s;
        // if robot is moving faster than the shot would need, clamp so we don't divide by negative speeds
        if (shotHorizontalSpeed < 0) {
            shotHorizontalSpeed = 0;
        }

        // lead angle: how far to lead the shot based on sideways motion vs horizontal shot speed
        // atan2(-tangential, shotSpeed) gives the angular correction to point into the moving target
        double leadAdjustmentDeg = Math.toDegrees(
                Math.atan2(-tangentialM_s, shotHorizontalSpeed)
        );

        // apply the lead so turret points where the target will be
        effectiveTurretDeg += leadAdjustmentDeg;

        // recompute the effective distance the ball will travel taking the actual shot speed and tangential motion into account
        effectiveRangeM = nominalShotTimeS *
                Math.hypot(tangentialM_s, shotHorizontalSpeed);

        // lookup base rpm and hood angle from empirical shot table for the effective range
        double baseRPM = ShotData.distanceToRPM.get(effectiveRangeM);
        double hoodDeg = ShotData.distanceToHoodAngle.get(effectiveRangeM);

        // feedforward for turret angular motion: combine robot yaw and the apparent angular rate due to tangential motion
        // note: tangential/range gives radians/sec approx, convert to deg/s
        double turretFF_deg_s = -(angularDeg_s + Math.toDegrees(tangentialM_s / rangeM));
        // feedforward for range: negative radial means robot moving toward target, so subtract
        double rangeFF_m_s    = -radialM_s;

        return new AdjustedShot(baseRPM, hoodDeg, effectiveTurretDeg,
                turretFF_deg_s, rangeFF_m_s);
    }
}