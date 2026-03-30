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
    private static final double NOMINAL_SHOT_TIME_S = 0.3; // see github issue #23 (https://github.com/Team293/Rebuilt/issues/23)
    private static ShotCompensation.AdjustedShot shotData = new ShotCompensation.AdjustedShot(0.0, 0.0, 0.0, 0.0, 0.0);

    private static Translation2d targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
    private final CommandSwerveDrivetrain drive;

    private static final double FIELD_WIDTH = 8.07; // meters
    private static final double FIELD_LENGTH = 16.54; // meters

    private static final double shuttlingXOffset = 1.5;
    private static final double shuttlingYOffset = 1.5;

    private boolean overrideRedAlliance = false;
    private boolean overrideBlueAlliance = false;
    

    public Targeting(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        setTargetingHub();
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));

        SmartDashboard.putBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        SmartDashboard.putBoolean("OverrideRedAlliance", overrideRedAlliance);
    }

    public static Translation2d differenceBetweenRobotAndTarget() {
        // calculate field-relative angle of the turret based on the turret motor position and the robot's heading
        // get pose of robo
        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d goalPose = targetPos;

        Translation2d turretPivotNow = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

        double staticDistance = goalPose.getDistance(turretPivotNow);
        double tof = ShotData.distanceToTOFConstant.get(staticDistance);
        // translate robot-center pose to the turret pivot location on the field

        ChassisSpeeds robotRelSpeeds = RobotContainer.getDrive().getState().Speeds;

        ChassisSpeeds speeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelSpeeds.vxMetersPerSecond,
                robotRelSpeeds.vyMetersPerSecond,
                robotRelSpeeds.omegaRadiansPerSecond,
                robotPose.getRotation()
            );

        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;

        Translation2d predictedRobotPos = robotPos.plus(
            new Translation2d(vx * tof, vy * tof)
        );

        Rotation2d predictedHeading =
            robotPose.getRotation().plus(
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * tof)
            );

        Translation2d predictedTurretPivot = Turret.TURRET_OFFSET_FROM_CENTER
            .rotateBy(predictedHeading)
            .plus(predictedRobotPos);

        Translation2d toGoalComp = goalPose.minus(predictedTurretPivot);
        
        Logger.recordOutput("Targeting/StaticDistance", staticDistance);
        Logger.recordOutput("Targeting/PredictedRobotPos", new Pose2d(predictedRobotPos, predictedHeading));
        Logger.recordOutput("Targeting/PredictedTurretPivot", new Pose2d(predictedTurretPivot, predictedHeading));
        Logger.recordOutput("Targeting/ToGoalCompensation", toGoalComp);
        Logger.recordOutput("Targeting/TimeOfFlight", tof);

        return toGoalComp;
    }
    
    /**
     * Periodically calculate the shot data given the target position and robot movement
     */
    @Override
    public void periodic() {
        Pose2d robotPose = drive.getPose();

        // compute the turret pivot location in field coordinates so ShotCompensation
        Translation2d turretPivot = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());
        Pose2d turretPivotPose = new Pose2d(turretPivot, robotPose.getRotation());

        if (DriverStation.isAutonomous()) {
            setTargetingHub();
        }

        Logger.recordOutput("Targeting/TurretPivot", turretPivotPose);
    }

    /**
     * Set the target location to center of the hub 
     */
    public void setTargetingHub() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SCORING mode")
        );
        Elastic.selectTab("Scoring Mode");

        overrideBlueAlliance = SmartDashboard.getBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        overrideRedAlliance = SmartDashboard.getBoolean("OverrideRedAlliance", overrideRedAlliance);

        // if (!DriverStation.getAlliance().isPresent() && isRedAlliance) {
        //     if (isRedAlliance) {
        //         targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        //     } else {
        //         targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        //     }
        //     return;
        // }

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
    public void setTargetingShuttleRight() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
        );
        Elastic.selectTab("Shuttling Mode");
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, 0 + shuttlingYOffset);
        }
    }

    public void setTargetingShuttleLeft() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
        );
        Elastic.selectTab("Shuttling Mode");
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, 0 + shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        }
    }
    
    /**
     * Returns the current shot data
     * @return ShotCompensation.AdjustedShot shot data to use
     */
    public static ShotCompensation.AdjustedShot getShotData() {
        return shotData;
    }
}
