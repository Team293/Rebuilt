package frc.robot.subsystems.climber;

import frc.lib.subsystem.SpikeSystem;

public class Climber extends SpikeSystem<ClimberIO.ClimberIOInputs> {
    private ClimberIO climberIO;

    public Climber() {
        super("Climber", new ClimberIO.ClimberIOInputs());
        this.climberIO = new ClimberIOTalonFX();
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.climberIO = new ClimberIOTalonFX();
        return useAsyncDataRefresher(climberIO);
    }
}
