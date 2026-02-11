package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DataUtils;
import frc.lib.LoggedTracer;

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
        return () -> {};
    }

    protected <T extends BaseIO<I>> Runnable useDataRefresher(
        T baseIO
    ) {
        return () -> baseIO.updateInputs(io);
    }

    public void onPeriodic() {}

    @Override
    public final void periodic() {
        dataRefreshTask.run();

        onPeriodic();
        LoggedTracer.record(getName());
    }
}