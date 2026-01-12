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

    private double timestamp = 0.0;
    private DataReaderAndLogger dataReaderAndLogger;
    private List<IODataRefresher> IODataRefreshers;

    public SubsystemDataProcessor(DataReaderAndLogger dataReaderAndLogger, IODataRefresher IODataRefresher) {
        this.dataReaderAndLogger = dataReaderAndLogger;
        IODataRefreshers = List.of(IODataRefresher);
    }

    public SubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger,
            IODataRefresher... refreshers) {
        this.dataReaderAndLogger = dataReaderAndLogger;
        IODataRefreshers = List.of(refreshers);
    }

    public void run() {
        while (true) {
            timestamp = System.currentTimeMillis();
            for (IODataRefresher IODataRefresher : IODataRefreshers) {
                IODataRefresher.refreshData();
            }

            dataReaderAndLogger.readAndLogDataFromIO();
            try {
                var difference = System.currentTimeMillis() - timestamp;
                if (difference < LOOP_TIME) {
                    Thread.sleep((long) (LOOP_TIME - difference));
                }
            } catch (InterruptedException e) {
            }
        }
    }
}