package frc.robot.subsystems.shooter;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private static final double SHOOTER_READY_THRESHOLD_RPS = 0.5; // RPS threshold to consider the shooter ready

    private ShooterIO shooterIO;
    private double targetRPS = 0.0; // Target rotations per second

    public Shooter() {
        super("Shooter", new ShooterIO.ShooterIOInputs());
    }

    /**
     *  Set rps and hood angle from adjusted shot data
     */ 
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

    /**
     * Sets up the data refresher for Shooter
     */
    @Override
    protected Runnable setupDataRefresher() {
        shooterIO = new ShooterIOTalonFX();
        return useAsyncDataRefresher(shooterIO);
    }

    /**
     * Checks if current rps of the motor is within the allowed error bounds 
     * @return if the target is within error bounds 
     */
    public boolean isAtTargetRPS() {
        return Math.abs(super.io.motorRPS - targetRPS) < SHOOTER_READY_THRESHOLD_RPS;
    }
}
