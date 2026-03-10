package frc.robot.subsystems.shooter;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.targeting.ShotCompensation;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private static final double SHOOTER_READY_THRESHOLD_RPS = 0.5; // RPS threshold to consider the shooter ready

    private ShooterIO shooterIO;
    private double targetRPS = 0.0; // Target rotations per second
    private boolean driverRequestingShooting = false; // Whether the driver is currently requesting to shoot

    public Shooter() {
        super("Shooter", new ShooterIOInputsAutoLogged());
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
            // shooterIO.setFlywheelVelocity(newTargetRPS);
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
     * Sets whether the driver is currently requesting to shoot. This can be used to determine if the shooter should be active or not.
      * @param isRequesting true if the driver is requesting to shoot, false otherwise
     */
    public void setDriverRequestingShooting(boolean isRequesting) {
        this.driverRequestingShooting = isRequesting;
    }

    /**
     * Returns whether the driver is currently requesting to shoot.
     * @return true if the driver is requesting to shoot, false otherwise
     */
    public boolean isDriverRequestingShooting() {
        return this.driverRequestingShooting;
    }
}
