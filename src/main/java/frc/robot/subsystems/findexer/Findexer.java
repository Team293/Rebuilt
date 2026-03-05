package frc.robot.subsystems.findexer;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.trigger.Trigger;

public class Findexer extends SpikeSystem<FindexerIO.FindexerIOInputs> {
    private static final double FEEDING_RPS = 20.0; // feeding velocity in rotations per second

    private final Trigger trigger;

    private FindexerIO findexerIO;

    public Findexer(Trigger trigger) {
        super("Findexer", new FindexerIO.FindexerIOInputs());

        this.trigger = trigger;
    }

    /**
     * Run the findexer if the indexer needs feeding (doesn't have a ball), otherwise stop it.
     */
    @Override
    public void onPeriodic() {
        // run the findexer if the indexer needs feeding (doesn't have a ball), otherwise stop it
        if (trigger.needsFeeding()) {
            findexerIO.setSpeed(FEEDING_RPS);
        } else {
            findexerIO.setSpeed(0.0);
        }
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
