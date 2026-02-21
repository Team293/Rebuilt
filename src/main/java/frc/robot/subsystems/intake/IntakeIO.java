package frc.robot.subsystems.intake;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.robot.subsystems.intake.Intake.IntakeState;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends BaseIO<IntakeIO.IntakeIOInputs> {
  @AutoLog
  class IntakeIOInputs extends BaseInputClass {
    public double intakeVelocityRPS = 0.0; // Intake velocity in Rotations Per Second
    public double intakeCurrentAmps = 0.0; // Intake Current in Amps
    public double deployVelocityRPS = 0.0; // Intake deploy motor Rotations Per Second
    public double deployCurrentAmps = 0.0; // Intake deploy motor Current in Amps
    public IntakeState intakeState = IntakeState.RETRACTED; // True if the intake is deployed, False otherwise
  }

  void setIntakeSpeed(double speed);
  void setDeploySpeed(double speed);
  IntakeState getIntakeState();
  void setIntakeState(IntakeState newState);
  double getDeployMotorCurrent();
}
