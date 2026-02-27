package frc.robot.subsystems.findexer;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.indexer.Indexer;

public class Findexer extends SpikeSystem<FindexerIO.FindexerIOInputs> {
    private static final double FEEDING_RPS = 20.0; // feeding velocity in rotations per second

    private final Indexer indexer;

    private FindexerIO findexerIO;

    public Findexer(Indexer indexer) {
        super("Findexer", new FindexerIO.FindexerIOInputs());

        this.indexer = indexer;
    }

    @Override
    public void onPeriodic() {
        if (indexer.needsFeeding()) {
            findexerIO.setSpeed(FEEDING_RPS);
        } else {
            findexerIO.setSpeed(0.0);
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.findexerIO = new FindexerIOTalonFX();
        return useAsyncDataRefresher(findexerIO);
    }
}
