package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    private Translation2d targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
    private final CommandSwerveDrivetrain drive;

    private static final double FIELD_WIDTH = 8.07; // meters
    private static final double FIELD_LENGTH = 16.54; // meters

    public Targeting(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        setTargetingHub();
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));
    }

    public static Translation2d differenceBetweenRobotAndTarget() {
        // calculate field-relative angle of the turret based on the turret motor position and the robot's heading
        // get pose of robo
        Pose2d robotPose = RobotContainer.getDrive().getPose();

        // translate robot-center pose to the turret pivot location on the field
        Translation2d turretPivot = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

        // vector from the turret pivot directly to the goal
        Translation2d goalPose = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        Translation2d toGoal = goalPose.minus(turretPivot);
        
        Logger.recordOutput("Turret/TurretPivot", new Pose2d(turretPivot, toGoal.getAngle()));
        return toGoal;
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

        shotData = ShotCompensation.compensateForMovement(
                turretPivotPose,
                drive.getState().Speeds,
                new Pose2d(targetPos, new Rotation2d()),
                NOMINAL_SHOT_TIME_S
        );

        Logger.recordOutput("Targeting/TurretPivot", turretPivotPose);
        
        Logger.recordOutput("Targeting/TargetPose", new Pose2d(targetPos, new Rotation2d()));
    }

    /**
     * Set the target location to center of the hub 
     */
    public void setTargetingHub() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SCORING mode")
        );
        Elastic.selectTab("Scoring Mode");
        if (DriverStation.getAlliance().isPresent() &&DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        } else {
            targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
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
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_WIDTH, FIELD_LENGTH);
        } else {
            targetPos = new Translation2d(0, 0);
        }
    }

    public void setTargetingShuttleLeft() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
        );
        Elastic.selectTab("Shuttling Mode");
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(0, FIELD_LENGTH);
        } else {
            targetPos = new Translation2d(FIELD_WIDTH, 0);
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
