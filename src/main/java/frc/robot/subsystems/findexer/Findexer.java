package frc.robot.subsystems.findexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.trigger.Trigger;
import org.littletonrobotics.junction.Logger;

public class Findexer extends SpikeSystem<FindexerIO.FindexerIOInputs> {
    private static final double MAX_TELEMETRY_AGE_SECONDS = 0.10;

    private final Trigger trigger;
    private final FindexerJamRecovery jamRecovery = new FindexerJamRecovery();

    private FindexerIO findexerIO;

    public Findexer(Trigger trigger) {
        super("Findexer", new FindexerIOInputsAutoLogged());

        this.trigger = trigger;
    }

    /**
     * Feed while requested, briefly reversing if the findexer remains stalled under load.
     */
    @Override
    public void onPeriodic() {
        boolean feedingRequested = DriverStation.isEnabled() && trigger.needsFeeding();
        boolean manualReverse = DriverStation.isEnabled() && trigger.getReverseTrigger();
        double targetRps;
        // The input processor updates these fields on another thread.
        synchronized (io) {
            double now = Timer.getFPGATimestamp();
            double age = now - io.sampleTimestampSeconds;
            boolean telemetryValid = io.motorConnected && age >= 0.0
                    && age <= MAX_TELEMETRY_AGE_SECONDS;
            targetRps = jamRecovery.calculate(now, feedingRequested, manualReverse,
                    telemetryValid, io.motorRps, io.statorCurrentAmps);
            Logger.recordOutput("Findexer/TelemetryValid", telemetryValid);
        }
        findexerIO.setSpeed(targetRps);
        Logger.recordOutput("Findexer/TargetRps", targetRps);
        Logger.recordOutput("Findexer/FeedingRequested", feedingRequested);
        Logger.recordOutput("Findexer/JamRecoveryState", jamRecovery.getState().name());
        Logger.recordOutput("Findexer/JamRecoveryAttempts", jamRecovery.getAttempts());
    }

    /** True when repeated clearing attempts have failed and feeding has been stopped. */
    public boolean isJamLockedOut() {
        return jamRecovery.getState() == FindexerJamRecovery.State.LOCKED_OUT;
    }

    /**
     * Sets up the data refresher for Findexer
     */
    @Override
    protected Runnable setupDataRefresher() {
        this.findexerIO = new FindexerIOTalonFX();
        return useAsyncDataRefresher(findexerIO);
    }
}
