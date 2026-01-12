package frc.lib;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;

public class DataUtils {
    public static <I extends BaseInputClass, T extends BaseIO<I> & IORefresher> void createDataLogger(
            I inputs, T baseIO
    ) {
        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
                () -> {
                    synchronized (inputs) {
                        baseIO.updateInputs(inputs);
                    }
                },
                baseIO
        );
    }
}