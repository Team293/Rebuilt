package frc.robot.subsystems.indexer;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO extends BaseIO<IndexerIO.IndexerIOInputs> {

    @AutoLog
    public static class IndexerIOInputs extends BaseInputClass {
        public double motorRps = 0.0;
    }

    abstract void setSpeed(double rps);
}