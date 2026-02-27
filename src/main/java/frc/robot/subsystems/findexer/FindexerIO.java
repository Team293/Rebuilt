package frc.robot.subsystems.findexer;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface FindexerIO extends BaseIO<FindexerIO.FindexerIOInputs>, IORefresher {
    @AutoLog
    public static class FindexerIOInputs extends BaseInputClass {
        public double motorRps = 0.0;
    }

    void setSpeed(double rps);
}
