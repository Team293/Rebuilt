package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.NotificationLevel;
import frc.lib.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;

public class Targeting extends SubsystemBase {
    // ==================== CONFIGURATION ====================
    
    /** Minimum robot speed (m/s) to enable moving shot compensation.
     *  Below this threshold, we use static aiming. */
    private static final double MIN_SPEED_FOR_MOVING_SHOT = 0.1;
    
    /** Whether to use physics-based moving shot compensation */
    private static boolean usePhysicsBasedMovingShot = true;
    
    // ==================== STATE ====================
    
    private static MovingShot.MovingShotResult lastMovingShotResult = null;

    private static Translation2d targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

    private static final double FIELD_WIDTH = 8.07; // meters
    private static final double FIELD_LENGTH = 16.54; // meters

    private static final double shuttlingXOffset = 2.0;
    private static final double shuttlingYOffset = 1.5;

    public static enum Target {
        HUB,
        SHUTTLE_RIGHT,
        SHUTTLE_LEFT
    }

    private Target currentTarget = Target.HUB;

    private boolean overrideRedAlliance = false;
    private boolean overrideBlueAlliance = false;
    
    public Targeting() {
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));

        SmartDashboard.putBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        SmartDashboard.putBoolean("OverrideRedAlliance", overrideRedAlliance);
        SmartDashboard.putBoolean("UsePhysicsMovingShot", usePhysicsBasedMovingShot);
    }

    /**
     * Calculate the vector from the turret pivot to the target, with optional
     * moving shot compensation using physics-based prediction.
     * 
     * @return Translation2d vector from turret to target (or compensated aim point)
     */
    public static Translation2d differenceBetweenRobotAndTarget() {
        Pose2d robotPose = RobotContainer.getDrive().getPose();
        
        // Calculate turret pivot position
        Translation2d turretPivot = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

        // Get robot velocity in field-relative coordinates
        ChassisSpeeds robotRelSpeeds = RobotContainer.getDrive().getState().Speeds;
        ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelSpeeds.vxMetersPerSecond,
                robotRelSpeeds.vyMetersPerSecond,
                robotRelSpeeds.omegaRadiansPerSecond,
                robotPose.getRotation()
        );

        double robotSpeed = Math.hypot(fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond);
        
        // Check if we should use moving shot compensation
        usePhysicsBasedMovingShot = SmartDashboard.getBoolean("UsePhysicsMovingShot", true);
        
        if (usePhysicsBasedMovingShot && robotSpeed > MIN_SPEED_FOR_MOVING_SHOT) {
            // Use physics-based moving shot calculation
            lastMovingShotResult = MovingShot.calculate(
                    robotPose,
                    Turret.TURRET_OFFSET_FROM_CENTER,
                    fieldRelSpeeds,
                    targetPos
            );
            
            // Convert the aim angle back to a vector for compatibility with existing code
            double aimAngleRad = Math.toRadians(lastMovingShotResult.turretAngleDeg());
            double effectiveDistance = lastMovingShotResult.effectiveDistanceM();
            
            // Create a vector that points in the aim direction with the effective distance
            Translation2d compensatedVector = new Translation2d(
                    effectiveDistance * Math.cos(aimAngleRad),
                    effectiveDistance * Math.sin(aimAngleRad)
            );
            
            Logger.recordOutput("Targeting/UsingMovingShot", true);
            Logger.recordOutput("Targeting/EffectiveDistance", effectiveDistance);
            Logger.recordOutput("Targeting/TimeOfFlight", lastMovingShotResult.timeOfFlightS());
            Logger.recordOutput("Targeting/LeadAngleDeg", lastMovingShotResult.leadAngleDeg());
            
            return compensatedVector;
        } else {
            // Static aiming - just point at the target
            Translation2d toGoal = targetPos.minus(turretPivot);
            
            Logger.recordOutput("Targeting/UsingMovingShot", false);
            Logger.recordOutput("Targeting/StaticDistance", toGoal.getNorm());
            
            lastMovingShotResult = null;
            return toGoal;
        }
    }
    
    /**
     * Get the effective distance to target, accounting for moving shot compensation.
     * This should be used for RPM and hood angle lookups.
     * 
     * @return Effective distance in meters
     */
    public static double getEffectiveDistance() {
        if (lastMovingShotResult != null && lastMovingShotResult.isValidShot()) {
            return lastMovingShotResult.effectiveDistanceM();
        }
        
        // Fall back to static distance
        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Translation2d turretPivot = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());
        return targetPos.minus(turretPivot).getNorm();
    }
    
    /**
     * Get the turret feedforward for angular velocity compensation.
     * Apply this to the turret to compensate for robot rotation during moving shots.
     * 
     * @return Turret feedforward in degrees per second
     */
    public static double getTurretFeedforward() {
        if (lastMovingShotResult != null && lastMovingShotResult.isValidShot()) {
            return lastMovingShotResult.turretAngularVelocityFFDegPerS();
        }
        return 0.0;
    }
    
    /**
     * Check if the current shot is valid (achievable with physics).
     */
    public static boolean isValidShot() {
        if (lastMovingShotResult != null) {
            return lastMovingShotResult.isValidShot();
        }
        return true; // Static shots are always "valid"
    }

    /**
     * Periodically calculate the shot data given the target position and robot movement
     */
    @Override
    public void periodic() {
        if (currentTarget == Target.HUB) {
            setPoseTargetingHub();
        } else if (currentTarget == Target.SHUTTLE_RIGHT) {
            setPoseTargetingShuttleRight();
        } else if (currentTarget == Target.SHUTTLE_LEFT) {
            setPoseTargetingShuttleLeft();
        }
    }

    /**
     * Set the target of the targeting subsystem. This will change the target position
     */
    public void setTarget(Target target) {
        currentTarget = target;
    }

    /**
     * Set the target location to center of the hub 
     */
    private void setPoseTargetingHub() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SCORING mode")
        );
        Elastic.selectTab("Scoring Mode");
        RobotContainer.getLEDController().switchPreset("hub");

        overrideBlueAlliance = SmartDashboard.getBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        overrideRedAlliance = SmartDashboard.getBoolean("OverrideRedAlliance", overrideRedAlliance);

        if (overrideRedAlliance) {
            targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
            return;
        }

        if (overrideBlueAlliance) {
            targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
            return;
        }


        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
            }
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
            }
        }
    }

    /**
     * Set the target location to 0, 0
     */
    private void setPoseTargetingShuttleRight() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
        );
        Elastic.selectTab("Shuttling Mode");
        RobotContainer.getLEDController().switchPreset("shuttle");

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, 0 + shuttlingYOffset);
        }
    }

    private void setPoseTargetingShuttleLeft() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
        );
        Elastic.selectTab("Shuttling Mode");
        RobotContainer.getLEDController().switchPreset("shuttle");

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, 0 + shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        }
    }

    /**
     * Returns the last moving shot result for advanced users
     */
    public static MovingShot.MovingShotResult getMovingShotResult() {
        return lastMovingShotResult;
    }
}
