package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.FieldConstants;
import frc.lib.SpikeController;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private static final double turretDeadbandDeg = 2.5; // degrees the turret must be within to be "on target"

    private TurretIOSensorInputs sensorData;
    private final CommandSwerveDrivetrain drive;
    private final SpikeController controller;

    private enum State { IDLE, TARGETING_HUB, TARGETING_SHUTTLE, ZEROING, MANUAL_CONTROL }

//    private final StateMachine<State> tsm;

    public static class TurretRequest {
        public double targetAngleDegrees;
    }

    public Turret(CommandSwerveDrivetrain drive, SpikeController controller) {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
        this.drive = drive;
        this.controller = controller;
    }

    @Override
    public void onPeriodic() {
//        tsm.tick();
        Logger.recordOutput("Turret/enc11", io.enc11);
        Logger.recordOutput("Turret/enc13", io.enc13);
        Logger.recordOutput("Turret/Angle", io.turretAngleDegrees);
        // double angleDiff = sensorData.getAngleOffsetFromPose(drive.getPose().getTranslation(), FieldConstants.Hub.innerCenterPoint.toTranslation2d());
        // Logger.recordOutput("Turret/AngleDiff", angleDiff);
        // double turretFieldAngleDeg = drive.getPose().getRotation().getDegrees() + io.turretAngleDegrees;
        // Pose2d turretPose = drive.getPose().rotateBy(new Rotation2d(TurretMath.toRad(turretFieldAngleDeg)));
        // Logger.recordOutput("Turret/TurretPose", turretPose);
        // sensorData.setTurretAngle(angleDiff);
        // double x = controller.getLeftX();
        // double y = -controller.getLeftY();

        // double angleDeg = Math.toDegrees(Math.atan2(y, x));

        // if (angleDeg < 0) {
        //     angleDeg += 360;
        // }

        // Logger.recordOutput("Turret/RequestedAngle", angleDeg);

        // LocalTime now = LocalTime.now();
        // int seconds = now.getSecond();
        // Logger.recordOutput("Turret/TimeSec", seconds);
        // double angle = seconds * 6;

        // Logger.recordOutput("Turret/RequestedAngle", angle);

        // TurretRequest req = new TurretRequest();
        // req.targetAngleDegrees = angleDeg;
        // runRequest(req);

    }

    public void runRequest(TurretRequest request) {
        sensorData.setTurretAngle(request.targetAngleDegrees);
    }

    @Override
    protected Runnable setupDataRefresher() {
        sensorData = new TurretIOSensorInputs(null);
        return useAsyncDataRefresher(sensorData);
    }
}
