package frc.robot.subsystems.intake;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends BaseIO<IntakeIO.IntakeIOInputs> {

    @AutoLog
    class IntakeIOInputs extends BaseInputClass {
        public double intakeVelocityRPS = 0.0; // Rotations per second
          public double intakeCurrentAmps = 0.0; // Amps
        public double deployVelocityRPS = 0.0; // Rotations per second
          public double deployCurrentAmps = 0.0;
        public boolean deployed = false; // whether the intake is deployed
    }

    void on(double speed);
    void off();
    void deploy();
    void retract();
}
