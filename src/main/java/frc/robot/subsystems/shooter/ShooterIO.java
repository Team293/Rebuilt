package frc.robot.subsystems.shooter;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends BaseIO<ShooterIO.ShooterIOInputs>, IORefresher {
    @AutoLog
    public static class ShooterIOInputs extends BaseInputClass {
        public double motorRPS = 0.0;
        public double hoodAngle = 15;
        public double hoodMotorPosition = 0.0;
        public double flywheelSetPointRPS = 0.0;
        public double hoodSetPointAngle = 0.0;
    }

    void setFlywheelVelocity(double rps);

    void setHoodAngle(double angle);
}
