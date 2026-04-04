package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.ShotData;
import frc.robot.subsystems.targeting.Targeting;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private static final double SHOOTER_READY_THRESHOLD_RPS = 2.0; // RPS threshold to consider the shooter ready

    private boolean readFromData = true;

    private ShooterIO shooterIO;
    private boolean spinUpFlywheel = false; 
    private boolean actuateHoodAndLaunch = false; 
    private boolean overrideStopShooting = false; // driver override to stop shooting and bring hood to  zero regardless of operator input

    public Shooter() {
        super("Shooter", new ShooterIOInputsAutoLogged());
        SmartDashboard.putNumber("TargetRPM", 0);
        SmartDashboard.putNumber("TargetHoodAngle", 0);
        SmartDashboard.putBoolean("ReadFromData", true);
    }

    /**
     *  Set rps and hood angle from adjusted shot data
     */ 
    @Override
    public void onPeriodic() {

        double distToTarget = getDistanceToTarget(); // distance in meters
        readFromData = SmartDashboard.getBoolean("ReadFromData", true);
        // double targetRPM = ShotData.distanceToRPM.get(distToTarget);
        // double hoodAngle = ShotData.distanceToHoodAngle.get(distToTarget);
        
        double targetRPM = 0;

        if (readFromData) {
            targetRPM = ShotData.distanceToRPM.get(distToTarget);
        } else {
            targetRPM = SmartDashboard.getNumber("TargetRPM", 0);
        }

        double hoodAngle = 0;
        if (readFromData) {
            hoodAngle = ShotData.distanceToHoodAngle.get(distToTarget);
        } else {
            hoodAngle = SmartDashboard.getNumber("TargetHoodAngle", 0);
        }

        if (io.isZeroing) {
            shooterIO.runZeroingHood();
        } else {
            if (overrideStopShooting || !actuateHoodAndLaunch) {
                shooterIO.setHoodAngle(0);
            } else {
                shooterIO.setHoodAngle(hoodAngle);
            }
        }

        // put to recovery mode if the driver is requesting to shoot
        // more direct control rather than smooth trajectory generation
        if (spinUpFlywheel) {
            shooterIO.setFlywheelVelocity(targetRPM / 60.0); // convert RPM to RPS
        } else {
            shooterIO.setFlywheelVelocity(0);
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
    @AutoLogOutput(key="Shooter/IsAtTargetRPS")
    public boolean isAtTargetRPS() {
        return Math.abs((super.io.flywheelVelocityRPS - 0.5) - io.flywheelSetPointRPS) < SHOOTER_READY_THRESHOLD_RPS;
    }

    /**
     * Returns the effective distance to target, accounting for moving shot compensation.
     * Uses Targeting.getEffectiveDistance() which compensates for robot velocity when shooting while moving.
     * @return effective distance in meters
     */
    public double getDistanceToTarget() {
        double effectiveDistance = Targeting.getEffectiveDistance();
        Logger.recordOutput("Targeting/DistanceToTarget", effectiveDistance);
        return effectiveDistance + io.distanceTrimMeters; // add distance trim to adjust the distance based on operator controller input
    }

    /**
     * Sets whether the driver is currently requesting to shoot. This can be used to determine if the shooter should be active or not.
      * @param isRequesting true if the driver is requesting to shoot, false otherwise
     */
    public void setDriverSpinUpFlywheel(boolean isRequesting) {
        this.spinUpFlywheel = isRequesting;
    }

    public void setActuateHoodAndLaunch(boolean isRequesting) {
        this.actuateHoodAndLaunch = isRequesting;
    }

    public void zeroHood() {
        shooterIO.zeroHood();
    }

    /**
     * Returns whether the driver is currently requesting to shoot.
     * @return true if the driver is requesting to shoot, false otherwise
     */
    public boolean isSpinUpFlywheel() {
        return this.spinUpFlywheel;
    }

    public boolean isActuatingHoodAndLaunching() {
        return this.actuateHoodAndLaunch;
    }

     /**
     * Changes the distance trim by a certain amount of meters. This is used to make minor adjustments to the distance based on operator controller input.
     * @param deltaDistance the amount of meters to change the distance trim by. Positive values add to the distance, and negative values subtract from the distance.
     */

    public void changeDistanceTrim(double deltaDistance) {
        shooterIO.changeDistanceTrim(deltaDistance);
    }

    public void setOverrideStopShooting(boolean overrideStopShooting) {
        this.overrideStopShooting = overrideStopShooting;
    }
}
