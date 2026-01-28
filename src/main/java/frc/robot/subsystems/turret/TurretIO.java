package frc.robot.subsystems.turret;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends BaseIO<TurretIO.TurretIOInputs> {

    @AutoLog
    public static class TurretIOInputs extends BaseInputClass {
        public double turretAngleDegrees = 0.0;
        public double offsetError = 0.0; // feed from camera, raw error value
    }
}
