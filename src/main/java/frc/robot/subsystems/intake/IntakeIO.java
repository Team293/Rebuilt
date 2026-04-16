package frc.robot.subsystems.intake;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends BaseIO<IntakeIO.IntakeIOInputs>, IORefresher {
  @AutoLog
  class IntakeIOInputs extends BaseInputClass {
    public double intakeVelocityRPS = 0.0; // Intake velocity in Rotations Per Second
    public double intakeCurrentAmps = 0.0; // Intake Current in Amps
    public double deployVelocityRPS = 0.0; // Intake deploy motor Rotations Per Second
    public double deployCurrentAmps = 0.0; // Intake deploy motor Current in Amps
  }

  void setIntakeSpeed(double speed);
  void setDeployServo(double position);
}
