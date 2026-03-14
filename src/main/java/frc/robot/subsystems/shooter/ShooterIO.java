package frc.robot.subsystems.shooter;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends BaseIO<ShooterIO.ShooterIOInputs>, IORefresher {
    @AutoLog
    public static class ShooterIOInputs extends BaseInputClass {
        public double motorRPS = 0.0; // Motor rotations per second
        public double hoodAngle = 15; // Hood angle in degrees from 
        public double hoodMotorPosition = 0.0; // TODO: position is in relation to what?
        public double flywheelSetPointRPS = 0.0; // Flywheel set point rotations per second
        public double hoodSetPointAngle = 0.0; // TODO: Hood setpoint in relation to what?
        public double hoodMotorCurrent = 0.0; // Current being drawn by the hood motor
        public double distanceTrim = 0.0; // minor adjustment to the returned distance based on operator controller input, in degrees
        public boolean isZeroing = false;
    }

    void setFlywheelVelocity(double rps, boolean isRecovery);
    void setHoodAngle(double angle);
    void runZeroingHood();
    void zeroHood();
    void changeDistanceTrim(double deltaDistance);
}
