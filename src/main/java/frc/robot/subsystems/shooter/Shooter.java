package frc.robot.subsystems.shooter;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private ShooterIO shooterIO;

    public Shooter() {
        super("Shooter", new ShooterIO.ShooterIOInputs());
    }

    @Override
    public void onPeriodic() {
        ShotCompensation.AdjustedShot shotData = Targeting.getShotData();

        if (shotData != null) {
            shooterIO.setHoodAngle(shotData.hoodAngleDeg());
            shooterIO.setFlywheelVelocity(shotData.rpm() / 60.0);
        };
    }

    @Override
    protected Runnable setupDataRefresher() {
        shooterIO = new ShooterIOTalonFX();
        return useAsyncDataRefresher(shooterIO);
    }
}
