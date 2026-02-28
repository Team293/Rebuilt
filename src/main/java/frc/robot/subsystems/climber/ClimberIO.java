package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;

public interface ClimberIO extends BaseIO<ClimberIO.ClimberIOInputs>, IORefresher {
    @AutoLog
    class ClimberIOInputs extends BaseInputClass {
        double climberPositionRotations = 0.0; // Climber position in rotation
        double climberCurrentAmps = 0.0; // Climber current in Amps
        double climberVelocityRPS = 0.0; // Climber velocity in Rotations Per Second
    }

    void setPosition(double positionRotations);
    double getPosition();
    void setVelocity(double velocityRotationsPerSecond);
    double getVelocity();
}
