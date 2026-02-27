package frc.robot.subsystems.shooter;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private static final double SHOOTER_READY_THRESHOLD_RPS = 0.5; // RPS threshold to consider the shooter ready

    private ShooterIO shooterIO;
    private double targetRPS = 0.0;

    public Shooter() {
        super("Shooter", new ShooterIO.ShooterIOInputs());
    }

    @Override
    public void onPeriodic() {
        ShotCompensation.AdjustedShot shotData = Targeting.getShotData();

        if (shotData != null) {
            double newTargetRPS = shotData.rpm() / 60.0;
            this.targetRPS = newTargetRPS;

            shooterIO.setHoodAngle(shotData.hoodAngleDeg());
            shooterIO.setFlywheelVelocity(newTargetRPS);
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        shooterIO = new ShooterIOTalonFX();
        return useAsyncDataRefresher(shooterIO);
    }

    public boolean isAtTargetRPS() {
        return Math.abs(super.io.motorRPS - targetRPS) < SHOOTER_READY_THRESHOLD_RPS;
    }
}
