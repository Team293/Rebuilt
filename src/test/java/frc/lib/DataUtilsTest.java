package frc.lib;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class DataUtilsTest {
    private static class Inputs extends BaseInputClass {
        int reading;
        int relatedReading;
        int setpoint;
    }

    private static class TestIO implements BaseIO<Inputs>, IORefresher {
        final Semaphore refreshed = new Semaphore(0);
        volatile Thread worker;
        int reading;
        int relatedReading;
        int setpoint = 42;
        int inputUpdates;
        Thread inputUpdateThread;

        @Override
        public void refreshData() {
            worker = Thread.currentThread();
            reading++;
            Thread.yield();
            relatedReading = reading;
            refreshed.release();
        }

        @Override
        public void updateInputs(Inputs inputs) {
            inputUpdateThread = Thread.currentThread();
            inputUpdates++;
            inputs.reading = reading;
            inputs.relatedReading = relatedReading;
            inputs.setpoint = setpoint;
        }
    }

    @Test
    void capturesOnMainThreadAndKeepsCycleInputsStableWhileWorkerRefreshes() throws Exception {
        Inputs inputs = new Inputs();
        TestIO hardware = new TestIO();
        Runnable captureInputs = DataUtils.createAsyncDataRefresher(inputs, hardware);
        try {
            assertTrue(hardware.refreshed.tryAcquire(2, TimeUnit.SECONDS));
            assertEquals(0, hardware.inputUpdates);
            assertEquals(0, inputs.reading);

            captureInputs.run();
            int cycleReading = inputs.reading;
            assertEquals(cycleReading, inputs.relatedReading);
            assertEquals(42, inputs.setpoint);
            assertSame(Thread.currentThread(), hardware.inputUpdateThread);
            assertNotSame(Thread.currentThread(), hardware.worker);

            hardware.setpoint = 88;
            hardware.refreshed.drainPermits();
            assertTrue(hardware.refreshed.tryAcquire(2, TimeUnit.SECONDS));
            // Logging and control can keep reading this cycle while hardware advances.
            assertEquals(cycleReading, inputs.reading);
            assertEquals(cycleReading, inputs.relatedReading);
            assertEquals(42, inputs.setpoint);
            assertEquals(1, hardware.inputUpdates);

            captureInputs.run();
            assertTrue(inputs.reading > cycleReading);
            assertEquals(inputs.reading, inputs.relatedReading);
            assertEquals(88, inputs.setpoint);
            assertEquals(2, hardware.inputUpdates);
        } finally {
            if (hardware.worker != null) {
                hardware.worker.interrupt();
                hardware.worker.join(2000);
                assertFalse(hardware.worker.isAlive());
            }
        }
    }
}
