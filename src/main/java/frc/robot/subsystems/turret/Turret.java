package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Elastic;
import frc.lib.FieldConstants;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.NotificationLevel;
import frc.lib.pubsub.PubResult;
import frc.lib.pubsub.PubTopic;
import frc.lib.pubsub.SimpleDataPub;
import frc.lib.pubsub.impl.PubBroker;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import frc.robot.subsystems.turret.calc.ShotCompensation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private final SimpleDataPub<ShotCompensation.AdjustedShot> shotCompensationPub;

    private final CommandSwerveDrivetrain drive;

    public enum State { TARGETING_HUB, TARGETING_SHUTTLE }

    private TurretIOTalonFX turretIO;
    private State currentState = State.TARGETING_HUB;
    private Translation2d targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

    public Turret(CommandSwerveDrivetrain drive) {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);

        this.shotCompensationPub = PubBroker.getOrCreate(PubTopic.SHOT_COMPENSATION);

        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        // compensate for robot movement
        Optional<PubResult<ShotCompensation.AdjustedShot>> pollRes = shotCompensationPub.poll();

        pollRes.ifPresent(res -> {
            if (!res.isInTimestamp()) {
                // we can add rejection logic if needed
                System.out.println("WARNING: Turret compensation data is stale!");
            }

            // set the turret angle to the compensated angle
            ShotCompensation.AdjustedShot compensatedShot = res.getData();

            this.turretIO.setTurretAngle(compensatedShot.turretAngleDeg());
        });

    }

    @Override
    protected Runnable setupDataRefresher() {
        this.turretIO = new TurretIOTalonFX(RobotContainer.getDrive());
        return useAsyncDataRefresher(this.turretIO);
    }

    public Translation2d getTargetPos() {
        return this.targetPos;
    }

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
}
