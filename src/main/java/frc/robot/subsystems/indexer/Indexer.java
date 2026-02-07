package main.java.frc.robot.subsystems.indexer;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SpikeSystem<IndexerIO.IndexerIOInputs> {
    private final static double indexerSpeed = 10.0;
    private final IndexerIOTalonFX indexerIO = new IndexerIOTalonFX();
    private DigitalInput beamBreak;

    public Indexer() {
        super("Indexer", new IndexerIO.IndexerIOInputs());
        beamBreak = new DigitalInput(2);
    }

    @Override
    public void onPeriodic() {
        if (beamBreak.get()) {
            indexerIO.setSpeed(0.0); 
        } else {
            indexerIO.setSpeed(indexerSpeed);
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        return useAsyncDataRefresher(indexerIO);
    }
}