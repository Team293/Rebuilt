package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/**
 * Physics-based shoot-on-the-move compensation system.
 * <br><br>
 * This class calculates the required turret angle and shot parameters to hit a target
 * while the robot is moving, using projectile motion physics rather than simple
 * linear prediction.
 * <br><br>
 * Key concepts:<br>
 * - The ball inherits the robot's velocity when shot<br>
 * - The ball follows a parabolic arc due to gravity<br>
 * - We need to "lead" the target to account for robot motion during flight<br>
 * - Turret feedforward compensates for robot rotation during aiming
 */
public class MovingShot {
    
    // ==================== PHYSICAL CONSTANTS ====================
    
    /** Gravity acceleration in m/s^2 */
    private static final double GRAVITY = 9.81;
    
    /** Height of the shooter exit above the ground in meters */
    private static final double SHOOTER_HEIGHT_METERS = 0.5; // Adjust based on robot
    
    /** Target (hub) height above the ground in meters */
    private static final double TARGET_HEIGHT_METERS = 2.64; // 2026 game target height - adjust as needed
    
    /** Flywheel radius in meters - used to convert RPM to ball exit velocity */
    private static final double FLYWHEEL_RADIUS_METERS = 0.127; // 2 inch radius = 0.051m
    
    /** Efficiency factor for flywheel to ball velocity transfer (typically 0.7-0.9) */
    private static final double FLYWHEEL_EFFICIENCY = 0.85;
    
    /** Maximum iterations for iterative TOF solver */
    private static final int MAX_ITERATIONS = 10;
    
    /** Convergence threshold for TOF solver in seconds */
    private static final double TOF_CONVERGENCE_THRESHOLD = 0.001;
    
    // ==================== RESULT RECORD ====================
    
    /**
     * Result of shoot-on-the-move calculations
     * @param turretAngleDeg Field-relative angle to aim the turret
     * @param effectiveDistanceM Effective distance to use for RPM/hood lookup
     * @param timeOfFlightS Calculated time of flight
     * @param turretAngularVelocityFFDegPerS Feedforward for turret angular velocity
     * @param leadAngleDeg Angular lead applied to compensate for motion
     * @param isValidShot Whether the calculated shot is physically achievable
     */
    public record MovingShotResult(
        double turretAngleDeg,
        double effectiveDistanceM,
        double timeOfFlightS,
        double turretAngularVelocityFFDegPerS,
        double leadAngleDeg,
        boolean isValidShot
    ) {}
    
    // ==================== PUBLIC API ====================
    
    /**
     * Calculate shoot-on-the-move parameters using physics-based prediction.
     * 
     * @param robotPose Current robot pose on the field
     * @param turretPivotOffset Offset from robot center to turret pivot point
     * @param fieldRelVelocity Robot velocity in field-relative coordinates
     * @param targetPosition Target position on the field
     * @return MovingShotResult with all calculated parameters
     */
    public static MovingShotResult calculate(
            Pose2d robotPose,
            Translation2d turretPivotOffset,
            ChassisSpeeds fieldRelVelocity,
            Translation2d targetPosition
    ) {
        // Calculate turret pivot position in field coordinates
        Translation2d turretPivot = turretPivotOffset
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());
        
        // Vector from turret to target (static - no motion compensation yet)
        Translation2d toTargetStatic = targetPosition.minus(turretPivot);
        double staticDistance = toTargetStatic.getNorm();
        double staticAngle = toTargetStatic.getAngle().getDegrees();
        
        // Get initial estimate of shot parameters for TOF calculation
        double estimatedRPM = ShotData.distanceToRPM.get(staticDistance);
        double hoodAngleDeg = ShotData.distanceToHoodAngle.get(staticDistance);
        
        // Calculate ball exit velocity from RPM
        double ballExitVelocity = rpmToBallVelocity(estimatedRPM);
        
        // Calculate time of flight using physics
        double timeOfFlight = calculateTimeOfFlight(
                staticDistance, 
                ballExitVelocity, 
                Math.toRadians(hoodAngleDeg)
        );
        
        // If TOF calculation failed, fall back to simple estimate
        if (timeOfFlight <= 0 || Double.isNaN(timeOfFlight)) {
            timeOfFlight = staticDistance / (ballExitVelocity * Math.cos(Math.toRadians(hoodAngleDeg)));
            if (timeOfFlight <= 0 || Double.isNaN(timeOfFlight)) {
                timeOfFlight = 0.8; // Last resort fallback
            }
        }
        
        // Now iterate to refine TOF with motion compensation
        // The ball inherits robot velocity, so we need to account for where the robot
        // (and thus the ball's starting point) will be, and where the target appears
        // relative to the moving ball
        
        double vx = fieldRelVelocity.vxMetersPerSecond;
        double vy = fieldRelVelocity.vyMetersPerSecond;
        double omega = fieldRelVelocity.omegaRadiansPerSecond;
        
        // Iteratively refine the solution
        double refinedTOF = timeOfFlight;
        Translation2d aimPoint = targetPosition;
        
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            // The ball inherits robot velocity, so relative to a ground-fixed frame,
            // the target appears to move opposite to robot velocity
            // But the ball also moves with robot velocity, so these cancel for the
            // HORIZONTAL component. However, the turret pivot moves, changing the angle.
            
            // - At t=0, ball is at turretPivot with velocity (ballVel + robotVel)
            // - At t=TOF, ball should be at targetPosition
            // - Ball travels: targetPosition - turretPivot in time TOF
            // - Ball velocity in field frame = launchVelocity (turret-relative) + robotVelocity
            
            // For the ball to hit the target:
            // turretPivot + (launchVelocity + robotVelocity) * TOF = targetPosition
            // launchVelocity * TOF = targetPosition - turretPivot - robotVelocity * TOF
            // launchVelocity * TOF = (targetPosition - robotVelocity * TOF) - turretPivot
            
            // So we aim at an "effective target" that is the real target minus robot motion
            aimPoint = targetPosition.minus(new Translation2d(vx * refinedTOF, vy * refinedTOF));
            
            // Recalculate distance and TOF to this aim point
            Translation2d toAimPoint = aimPoint.minus(turretPivot);
            double aimDistance = toAimPoint.getNorm();
            
            // Update shot parameters for new distance
            double newRPM = ShotData.distanceToRPM.get(aimDistance);
            double newHoodAngle = ShotData.distanceToHoodAngle.get(aimDistance);
            double newBallVelocity = rpmToBallVelocity(newRPM);
            
            double newTOF = calculateTimeOfFlight(
                    aimDistance,
                    newBallVelocity,
                    Math.toRadians(newHoodAngle)
            );
            
            if (newTOF <= 0 || Double.isNaN(newTOF)) {
                break; // Keep previous TOF
            }
            
            // Check for convergence
            if (Math.abs(newTOF - refinedTOF) < TOF_CONVERGENCE_THRESHOLD) {
                refinedTOF = newTOF;
                break;
            }
            
            refinedTOF = newTOF;
        }
        
        // Calculate final aim direction
        Translation2d finalAimVector = aimPoint.minus(turretPivot);
        double aimAngleDeg = finalAimVector.getAngle().getDegrees();
        double effectiveDistance = finalAimVector.getNorm();
        
        // Calculate lead angle (how much we're leading the shot)
        double leadAngleDeg = aimAngleDeg - staticAngle;
        // Normalize to [-180, 180]
        if (leadAngleDeg > 180) leadAngleDeg -= 360;
        if (leadAngleDeg < -180) leadAngleDeg += 360;
        
        // Calculate turret feedforward to compensate for robot rotation
        // The turret needs to counter-rotate to maintain field-relative aim
        // Plus compensate for the apparent angular motion of the target
        double apparentAngularRate = calculateApparentAngularRate(
                turretPivot, targetPosition, vx, vy
        );
        double turretFF = -(Math.toDegrees(omega) + apparentAngularRate);
        
        // Validate the shot
        boolean isValid = effectiveDistance > 0.5 && 
                          effectiveDistance < 20.0 && 
                          refinedTOF > 0 && 
                          refinedTOF < 3.0;
        
        // Log debug info
        Logger.recordOutput("MovingShot/StaticDistance", staticDistance);
        Logger.recordOutput("MovingShot/EffectiveDistance", effectiveDistance);
        Logger.recordOutput("MovingShot/TimeOfFlight", refinedTOF);
        Logger.recordOutput("MovingShot/LeadAngleDeg", leadAngleDeg);
        Logger.recordOutput("MovingShot/TurretFF", turretFF);
        Logger.recordOutput("MovingShot/AimPoint", new Pose2d(aimPoint, new Rotation2d()));
        
        return new MovingShotResult(
                aimAngleDeg,
                effectiveDistance,
                refinedTOF,
                turretFF,
                leadAngleDeg,
                isValid
        );
    }
    
    // ==================== PHYSICS CALCULATIONS ====================
    
    /**
     * Convert flywheel RPM to ball exit velocity in m/s
     */
    private static double rpmToBallVelocity(double rpm) {
        // v = omega * r * efficiency
        // omega = rpm * 2pi / 60
        double omegaRadPerSec = rpm * 2 * Math.PI / 60.0;
        return omegaRadPerSec * FLYWHEEL_RADIUS_METERS * FLYWHEEL_EFFICIENCY;
    }
    
    /**
     * Calculate time of flight using projectile motion physics.
     * <br>
     * Uses the equations:
     * - x(t) = v0 * cos(theta) * t
     * - y(t) = h0 + v0 * sin(theta) * t - 0.5 * g * t^2
     * <br>
     * @param horizontalDistance Distance to target in meters
     * @param exitVelocity Ball exit velocity in m/s
     * @param launchAngleRad Launch angle in radians (from horizontal)
     * @return Time of flight in seconds, or -1 if no valid solution
     */
    private static double calculateTimeOfFlight(
            double horizontalDistance,
            double exitVelocity,
            double launchAngleRad
    ) {
        double vx = exitVelocity * Math.cos(launchAngleRad);
        double vy = exitVelocity * Math.sin(launchAngleRad);
        
        if (vx <= 0) {
            return -1; // Can't reach target with zero or negative horizontal velocity
        }
        
        // Time to travel horizontal distance
        double t = horizontalDistance / vx;
        
        // Verify the ball reaches the target height at this time
        // y(t) = h0 + vy*t - 0.5*g*t^2
        double actualHeight = vy * t - 0.5 * GRAVITY * t * t;
        
        // If we're not reaching target height, this isn't a valid trajectory
        // But we'll return the time anyway and let the caller decide
        // A more sophisticated approach would solve for the correct launch angle
        
        Logger.recordOutput("MovingShot/PredictedHeight", actualHeight + SHOOTER_HEIGHT_METERS);
        Logger.recordOutput("MovingShot/TargetHeight", TARGET_HEIGHT_METERS);
        
        return t;
    }
    
    /**
     * Calculate the apparent angular rate of the target due to robot translation.
     * <br>
     * When the robot moves tangentially to the target, the target appears to
     * move angularly even though it's stationary.
     * 
     * @return Apparent angular rate in degrees per second
     */
    private static double calculateApparentAngularRate(
            Translation2d turretPivot,
            Translation2d target,
            double vx,
            double vy
    ) {
        Translation2d toTarget = target.minus(turretPivot);
        double distance = toTarget.getNorm();
        
        if (distance < 0.1) {
            return 0; // Too close, avoid division issues
        }
        
        // Angle to target
        Rotation2d angleToTarget = toTarget.getAngle();
        
        // Robot velocity vector
        Translation2d velocity = new Translation2d(vx, vy);
        
        // Tangential component of velocity (perpendicular to line-of-sight)
        // Rotate velocity into target frame, take Y component
        Translation2d velInTargetFrame = velocity.rotateBy(angleToTarget.unaryMinus());
        double tangentialVel = velInTargetFrame.getY();
        
        // Angular rate = tangential velocity / distance
        double angularRateRad = tangentialVel / distance;
        
        return Math.toDegrees(angularRateRad);
    }
}
