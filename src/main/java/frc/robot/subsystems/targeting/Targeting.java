package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.NotificationLevel;
import frc.lib.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Targeting extends SubsystemBase {
    private static final double NOMINAL_SHOT_TIME_S = 0.3; // see github issue #23 (https://github.com/Team293/Rebuilt/issues/23)
    private static ShotCompensation.AdjustedShot shotData = new ShotCompensation.AdjustedShot(0.0, 0.0, 0.0, 0.0, 0.0);

    private Translation2d targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
    private final CommandSwerveDrivetrain drive;

    public Targeting(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        setTargetingHub();
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));
    }
    
    /**
     * Periodically calculate the shot data given the target position and robot movement
     */
    @Override
    public void periodic() {
        // calculate the adjusted shot parameters based on the current robot movement and the turret's target position
        shotData = ShotCompensation.compensateForMovement(
                drive.getPose(),
                drive.getState().Speeds,
                new Pose2d(targetPos, new Rotation2d()),
                NOMINAL_SHOT_TIME_S
        );
    }

    /**
     * Set the target location to center of the hub 
     */
    public void setTargetingHub() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SCORING mode")
        );
        Elastic.selectTab("Scoring Mode");
        targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    }

    /**
     * Set the target location to 0, 0
     */
    public void setTargetingShuttle() {
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
        );
        Elastic.selectTab("Shuttling Mode");
        targetPos = new Translation2d(0, 0);
    }
    
    /**
     * Returns the current shot data
     * @return ShotCompensation.AdjustedShot shot data to use
     */
    public static ShotCompensation.AdjustedShot getShotData() {
        return shotData;
    }
}
