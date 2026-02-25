package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Elastic;
import frc.lib.FieldConstants;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.NotificationLevel;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import frc.robot.subsystems.turret.calc.ShotCompensation;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private TurretIOTalonFX turretIO;
    private final CommandSwerveDrivetrain drive;

    public enum State { TARGETING_HUB, TARGETING_SHUTTLE }

    private State currentState = State.TARGETING_HUB;

    private Translation2d targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

    public void setTargetingHub() {
        if (currentState != State.TARGETING_HUB) {
            Elastic.sendNotification(
                    new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SCORING mode")
            );
            Elastic.selectTab("Scoring Mode");
            targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
            currentState = State.TARGETING_HUB;
        }
    }

    public void setTargetingShuttle() {
        if (currentState != State.TARGETING_SHUTTLE) {
            Elastic.sendNotification(
                    new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
            );
            Elastic.selectTab("Shuttling Mode");
            targetPos = new Translation2d(0, 0);
            currentState = State.TARGETING_SHUTTLE;
        }
    }

    public Turret(CommandSwerveDrivetrain drive) {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        // compensate for robot movement
        ShotCompensation.AdjustedShot targetAngleCompensated = ShotCompensation.compensateForMovement(
                drive.getPose(),
                drive.getState().Speeds,
                new Pose2d(this.targetPos, new Rotation2d()),
                0.3 // see github issue #23 (https://github.com/Team293/Rebuilt/issues/23)
        );

        // set the turret angle to the compensated angle
        this.turretIO.setTurretAngle(targetAngleCompensated.turretAngleDeg());
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.turretIO = new TurretIOTalonFX(RobotContainer.getDrive());
        return useAsyncDataRefresher(this.turretIO);
    }
}
