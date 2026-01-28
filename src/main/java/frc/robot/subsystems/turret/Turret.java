package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.FieldConstants;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem {
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
    private final TurretIOSensorInputs sensorData;
    private final CommandSwerveDrivetrain drive;

    public Turret(CommandSwerveDrivetrain drive) {
        super("Turret");
        this.drive = drive;
        this.sensorData = new TurretIOSensorInputs(null); // TODO: add camera
    }

    @Override
    public void onPeriodic() {
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
    }

    @Override
    protected Runnable setupDataRefresher() {
        return useAsyncDataRefresher(inputs, sensorData);
    }

    public double getAngleOffsetFromHub() {
        Translation2d hubLocation = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        // get robot position
        Translation2d robotPosition = drive.getPose().getTranslation();
        // calculate angle difference between turret and hub
        double angleToHub = Math.toDegrees(Math.atan2(hubLocation.getY() - robotPosition.getY(),
                hubLocation.getX() - robotPosition.getX()));
        double turretAngle = inputs.turretAngleDegrees;
        double angleOffset = angleToHub - turretAngle;
        // normalize angle to [-180, 180]
        angleOffset = ((angleOffset + 180) % 360) - 180;
        return angleOffset;
    }
}
