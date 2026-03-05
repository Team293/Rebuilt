package frc.robot.subsystems.shooter;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private static final double SHOOTER_READY_THRESHOLD_RPS = 0.5; // RPS threshold to consider the shooter ready

    private ShooterIO shooterIO;
    private double targetRPS = 0.0; // Target rotations per second
    private boolean driverRequestingShooting = false; // Whether the driver is currently requesting to shoot

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

    /**
     * Sets the driver requesting shooting to true, indicating that the driver is currently requesting to shoot. This should be called when the driver presses the shoot button.
     */
    public void setDriverRequestingShootingTrue() {
        this.driverRequestingShooting = true;
    }

    /**
    * Sets the driver requesting shooting to false, indicating that the driver is no longer requesting to shoot. This should be called when the driver releases the shoot button.
    */
    public void setDriverRequestingShootingFalse() {
        this.driverRequestingShooting = false;
    }

    /**
     * Returns whether the driver is currently requesting to shoot.
     * @return true if the driver is requesting to shoot, false otherwise
     */
    public boolean isDriverRequestingShooting() {
        return this.driverRequestingShooting;
    }
}
