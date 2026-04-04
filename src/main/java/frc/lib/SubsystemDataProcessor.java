package frc.lib;

import java.util.List;
import java.util.concurrent.locks.LockSupport;

public class SubsystemDataProcessor implements Runnable {
    public interface DataReaderAndLogger {
        void readAndLogDataFromIO();
    }

    public interface IODataRefresher {
        void refreshData();
    }

    public static final int LOOP_TIME_MS = 20;
    private static final long LOOP_TIME_NS = LOOP_TIME_MS * 1_000_000L;

    public static void createAndStartSubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger, IODataRefresher IODataRefresher) {
        new Thread(new SubsystemDataProcessor(dataReaderAndLogger, IODataRefresher)).start();
    }

    public static void createAndStartSubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger, IODataRefresher... refreshers) {
        new Thread(new SubsystemDataProcessor(dataReaderAndLogger, refreshers)).start();
    }

    private DataReaderAndLogger dataReaderAndLogger;
    private List<IODataRefresher> IODataRefreshers;

    public SubsystemDataProcessor(DataReaderAndLogger dataReaderAndLogger, IODataRefresher IODataRefresher) {
        this.dataReaderAndLogger = dataReaderAndLogger;

        if (IODataRefresher == null) {
            IODataRefreshers = List.of();
        } else {
            IODataRefreshers = List.of(IODataRefresher);
        }
    }

    public SubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger,
            IODataRefresher... refreshers) {                 
        this.dataReaderAndLogger = dataReaderAndLogger;
        IODataRefreshers = List.of(refreshers);
    }

    @Override
    public void run() {
        // Calculate absolute deadline for next iteration
        long nextDeadlineNs = System.nanoTime() + LOOP_TIME_NS;
        
        while (!Thread.currentThread().isInterrupted()) {
            for (IODataRefresher IODataRefresher : IODataRefreshers) {
                IODataRefresher.refreshData();
            }

            dataReaderAndLogger.readAndLogDataFromIO();

            // Use absolute time-based parking for precise timing
            // LockSupport.parkNanos uses absolute deadline avoiding drift accumulation
            long sleepNs = nextDeadlineNs - System.nanoTime();
            if (sleepNs > 0) {
                LockSupport.parkNanos(sleepNs);
            }
            
            // Calculate next deadline based on previous deadline (not current time)
            // This prevents drift accumulation over time
            nextDeadlineNs += LOOP_TIME_NS;
            
            // If we've fallen behind by more than one full period, reset the deadline
            // to prevent trying to "catch up" by running many iterations rapidly
            long currentNs = System.nanoTime();
            if (nextDeadlineNs < currentNs - LOOP_TIME_NS) {
                nextDeadlineNs = currentNs + LOOP_TIME_NS;
            }
        }
    }
}