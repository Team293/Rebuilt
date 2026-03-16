package frc.lib;

import java.util.List;

public class SubsystemDataProcessor implements Runnable {
    public interface DataReaderAndLogger {
        void readAndLogDataFromIO();
    }

    public interface IODataRefresher {
        void refreshData();
    }

    public static final int LOOP_TIME = 20;

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
        while (!Thread.currentThread().isInterrupted()) {
            long startNs = System.nanoTime();
            for (IODataRefresher IODataRefresher : IODataRefreshers) {
                IODataRefresher.refreshData();
            }

            dataReaderAndLogger.readAndLogDataFromIO();

            long elapsedMs = (System.nanoTime() - startNs) / 1_000_000L;
            long sleepMs = LOOP_TIME - elapsedMs;
            if (sleepMs > 0) {
                try {
                    Thread.sleep(sleepMs);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // restore interrupt flag so the loop exits cleanly
                }
            }
        }
    }
}