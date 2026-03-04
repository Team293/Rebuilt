package frc.robot.subsystems.trigger;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface TriggerIO extends BaseIO<TriggerIO.TriggerIOInputs>, IORefresher {

    @AutoLog
    public static class TriggerIOInputs extends BaseInputClass {
        public double motorRps = 0.0;
        public boolean proximitySensor = false;
    }

    abstract void setSpeed(double rps);
}