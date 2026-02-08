package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.FieldConstants;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.calc.ShotCompensation;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private static final double turretDeadbandDeg = 2.5; // degrees the turret must be within to be "on target"

    private TurretIOSensorInputs sensorData;
    private final CommandSwerveDrivetrain drive;

    private enum State { IDLE, TARGETING_HUB, TARGETING_SHUTTLE, ZEROING, MANUAL_CONTROL }

    private Translation2d target = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

//    private final StateMachine<State> tsm;

    public static class TurretRequest {
        public double targetAngleDegrees;
    }

    public Turret(CommandSwerveDrivetrain drive) {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
//        tsm.tick();
        ShotCompensation.AdjustedShot shot = ShotCompensation.compensateForMovement(drive.getPose(), drive.getState().Speeds, new Pose2d(target, new Rotation2d()), 0.6);
    }

    public void runRequest(TurretRequest request) {
        double clampedAngle = MathUtil.clamp(
            request.targetAngleDegrees,
            -180.0,
            180.0
        );
        sensorData.setTurretAngle(clampedAngle);
    }

    @Override
    protected Runnable setupDataRefresher() {
        sensorData = new TurretIOSensorInputs(null);
        return useAsyncDataRefresher(sensorData);
    }
}
