package frc.robot.subsystems.intake;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends BaseIO<IntakeIO.IntakeIOInputs> {

    @AutoLog
    public static class IntakeIOInputs extends BaseInputClass {
        public double motorRps = 0.0;
    }

    abstract void setSpeed(double rps);
}
