package frc.robot.subsystems.shooter;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends BaseIO<ShooterIO.ShooterIOInputs>, IORefresher {
    @AutoLog
    public static class ShooterIOInputs extends BaseInputClass {
        public double flywheelVelocityRPS = 0.0; // measured flywheel speed, in rotations per second
        public double hoodAngleDeg = 15.0; // measured hood angle, in degrees [15, 45]
        public double flywheelSetPointRPS = 0.0; // commanded flywheel speed setpoint, in rotations per second
        public double hoodAngleSetPointDeg = 0.0; // commanded hood angle setpoint, in degrees [15, 45]
        public double hoodMotorCurrentAmps = 0.0; // measured hood motor supply current, in amps
        public double distanceTrimMeters = 0.0; // operator fine-trim added to the target distance, in meters
        public boolean isZeroing = false; // true while the hood is running its zeroing routine
    }

    void setFlywheelVelocity(double rps);
    void setHoodAngle(double angle);
    void runZeroingHood();
    void zeroHood();
    void changeDistanceTrim(double deltaDistance);
    void setDistanceTrim(double deltaDistance);
    double getDistanceTrim();
}
