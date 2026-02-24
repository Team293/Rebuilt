package frc.robot.subsystems.shooter;

import frc.lib.subsystem.SpikeSystem;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {

    private final ShooterIO shooterInputs;

    public Shooter(String name, ShooterIO.ShooterIOInputs io) {
        super(name, io);
    }

    @Override
    protected Runnable setupDataRefresher() {
        shooterInputs =
        return useAsyncDataRefresher();
    }
}
