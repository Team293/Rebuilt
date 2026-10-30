package frc.lib;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;

public class DataUtils {
    /** Refresh hardware on a worker, then capture inputs once per main-thread loop. */
    public static <I extends BaseInputClass, T extends BaseIO<I> & IORefresher> Runnable createAsyncDataRefresher(
            I inputs, T baseIO
    ) {
        Object refreshLock = new Object();
        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
                () -> {},
                () -> {
                    synchronized (refreshLock) {
                        baseIO.refreshData();
                    }
                }
        );
        return () -> {
            synchronized (refreshLock) {
                baseIO.updateInputs(inputs);
            }
        };
    }
}
