package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.targeting.ShotData;
import frc.robot.subsystems.targeting.Targeting;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private static final double SHOOTER_READY_THRESHOLD_RPS = 2.0; // RPS threshold to consider the shooter ready

    private boolean readFromData = true;

    private ShooterIO shooterIO;
    private boolean driverRequestingShooting = false; // Whether the driver is currently requesting to shoot

    public Shooter() {
        super("Shooter", new ShooterIOInputsAutoLogged());
        // SmartDashboard.putNumber("TargetRPM", 0);
        // SmartDashboard.putNumber("TargetHoodAngle", 0);
        // SmartDashboard.putBoolean("ReadFromData", true);
    }

    /**
     *  Set rps and hood angle from adjusted shot data
     */ 
    @Override
    public void onPeriodic() {
        double distToTarget = getDistanceToTarget(); // distance in meters
        // double targetRPM = ShotData.distanceToRPM.get(distToTarget);
        // double hoodAngle = ShotData.distanceToHoodAngle.get(distToTarget);
        
        double targetRPM = 0;

        if (readFromData) {
            targetRPM = ShotData.distanceToRPM.get(distToTarget);
        } else {
            // targetRPM = SmartDashboard.getNumber("TargetRPM", 0);
        }

        double hoodAngle = 0;
        if (readFromData) {
            hoodAngle = ShotData.distanceToHoodAngle.get(distToTarget);
        } else {
            // hoodAngle = SmartDashboard.getNumber("TargetHoodAngle", 0);
        }

        if (io.isZeroing) {
            shooterIO.runZeroingHood();
        } else {
            shooterIO.setHoodAngle(hoodAngle);
        }

        // put to recovery mode if the driver is requesting to shoot
        // more direct control rather than smooth trajectory generation
        shooterIO.setFlywheelVelocity(targetRPM / 60.0, driverRequestingShooting); // convert RPM to RPS
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
     * Returns the distance from the center of the turret to the target in meters
     * @return
     */
    public double getDistanceToTarget() {
        Translation2d toGoal = Targeting.differenceBetweenRobotAndTarget();
        return toGoal.getNorm() + io.distanceTrimMeters;
    }

    /**
     * Sets whether the driver is currently requesting to shoot. This can be used to determine if the shooter should be active or not.
      * @param isRequesting true if the driver is requesting to shoot, false otherwise
     */
    public void setDriverRequestingShooting(boolean isRequesting) {
        this.driverRequestingShooting = isRequesting;
    }

    public void zeroHood() {
        shooterIO.zeroHood();
    }

    /**
     * Returns whether the driver is currently requesting to shoot.
     * @return true if the driver is requesting to shoot, false otherwise
     */
    public boolean isDriverRequestingShooting() {
        return this.driverRequestingShooting;
    }

    public void changeDistanceTrim(double deltaDistance) {
        shooterIO.changeDistanceTrim(deltaDistance);
    }
}
