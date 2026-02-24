package frc.robot.subsystems.indexer;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SpikeSystem<IndexerIO.IndexerIOInputs> {
    private final static double indexerSpeed = 10.0; // Rotations per second

    private IndexerIO indexerIO;
    private DigitalInput proximitySensor;

    public Indexer(int channel) {
        super("Indexer", new IndexerIO.IndexerIOInputs());
        proximitySensor = new DigitalInput(channel);
    }

    // Activate motor if proximity sensor detects a ball in the indexer
    @Override
    public void onPeriodic() {
        if (proximitySensor.get()) {
            indexerIO.setSpeed(0.0); 
        } else {
            indexerIO.setSpeed(indexerSpeed);
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.indexerIO = new IndexerIOTalonFX();
        return useAsyncDataRefresher(indexerIO);
    }
}