package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Elastic;
import frc.lib.FieldConstants;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.NotificationLevel;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import frc.robot.subsystems.turret.calc.ShotCompensation;
import frc.robot.subsystems.turret.calc.TurretMath;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private TurretIOSensorInputs sensorData;
    private final CommandSwerveDrivetrain drive;

    public enum State { TARGETING_HUB, TARGETING_SHUTTLE }

    private State currentState = State.TARGETING_HUB;

    private Translation2d targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

    public void switchState(State state) {
        currentState = state;
    }

    public Turret(CommandSwerveDrivetrain drive) {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        switch (currentState) {
            case TARGETING_HUB -> {
                Elastic.sendNotification(
                        new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SCORING mode")
                );
                Elastic.selectTab("Scoring Mode");
                targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
            }
            case TARGETING_SHUTTLE -> {
                Elastic.sendNotification(
                        new Notification(NotificationLevel.INFO, "Switched Modes", "Switched modes to SHUTTLING mode")
                );
                Elastic.selectTab("Shuttling Mode");
                targetPos = new Translation2d();
            }
        }

        Logger.recordOutput("Turret/PinionEncoder", io.pinionEncoder);
        Logger.recordOutput("Turret/FollowerEncoder", io.followerEncoder);
        Logger.recordOutput("Turret/Angle", io.turretAngleDegrees);
        double turretFieldAngleDeg = io.turretAngleDegrees;
        Pose2d turretPose = new Pose2d(drive.getPose().getTranslation(), new Rotation2d(TurretMath.toRad(turretFieldAngleDeg)));
        Logger.recordOutput("Turret/TurretPose", turretPose);

        ShotCompensation.AdjustedShot targetAngleCompensated = ShotCompensation.compensateForMovement(
                drive.getPose(),
                drive.getState().Speeds,
                new Pose2d(targetPos, new Rotation2d()),
                0.3 // see #23
        );

        sensorData.setTurretAngle(targetAngleCompensated.turretAngleDeg());
        Logger.recordOutput("Turret/TargetOffset", targetAngleCompensated.turretAngleDeg());
    }

    @Override
    protected Runnable setupDataRefresher() {
        sensorData = new TurretIOSensorInputs();
        return useAsyncDataRefresher(sensorData);
    }
}
