package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Elastic;
import frc.lib.FieldConstants;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.NotificationLevel;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;

import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.turret.calc.ShotCompensation;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private static final double TURRET_ANGLE_THRESHOLD_DEG = 3.0; // degrees within target angle to be considered "at target"

    public enum State { TARGETING_HUB, TARGETING_SHUTTLE }

    private TurretIOTalonFX turretIO;
    private State currentState = State.TARGETING_HUB;
    private Translation2d targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    private double targetAngleDeg = 0.0;

    public Turret() {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
    }

    @Override
    public void onPeriodic() {
        // compensate for robot movement
        ShotCompensation.AdjustedShot shotData = Targeting.getShotData();

        if (shotData != null) {
            double newTargetAngleDeg = shotData.turretAngleDeg();
            this.targetAngleDeg = newTargetAngleDeg;

            this.turretIO.setTurretAngle(newTargetAngleDeg);
        }
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

    public boolean isAtTargetAngle() {
        double angleError = Math.abs(super.io.turretAngleDegrees - targetAngleDeg);
        return angleError < TURRET_ANGLE_THRESHOLD_DEG;
    }
}
