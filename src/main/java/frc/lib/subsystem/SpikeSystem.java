package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DataUtils;
import frc.lib.LoggedTracer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class SpikeSystem<I extends BaseInputClass> extends SubsystemBase {
    private final Runnable dataRefreshTask;
    protected final I io;
    
    public SpikeSystem(String name, I io) {
        super(name);
        this.io = io;
        this.dataRefreshTask = setupDataRefresher();
    }

    protected abstract Runnable setupDataRefresher();

    protected <T extends BaseIO<I> & IORefresher> Runnable useAsyncDataRefresher(
        T baseIO
    ) {
        DataUtils.createDataLogger(io, baseIO);
        return () -> {
            if (io instanceof LoggableInputs loggable) {
                synchronized (io) {
                    Logger.processInputs(getName(), loggable);
                }
            }
        };
    }

    protected <T extends BaseIO<I>> Runnable useDataRefresher(
        T baseIO
    ) {
        return () -> {
            baseIO.updateInputs(io);
            if (io instanceof LoggableInputs loggable) {
                Logger.processInputs(getName(), loggable);
            }
        };
    }

    public void onPeriodic() {}

    @Override
    public final void periodic() {
        LoggedTracer.reset();
        dataRefreshTask.run();

        onPeriodic();
        LoggedTracer.record(getName());
    }
}