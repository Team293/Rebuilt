package frc.robot.subsystems.turret;


import frc.lib.LowPassFilter;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.targeting.Targeting;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    // ==================== TURRET CONFIGURATION CONSTANTS ====================
    
    /** Degrees within which we consider the turret to be aimed at the target (+/-) */
    public static final double TURRET_AIMING_TOLERANCE_DEGREES = 5.0;
    
    /** Maximum turret rotation range in degrees (+/-). Set to 200 for +/- 200 degrees of travel. */
    public static final double TURRET_MAX_ANGLE_DEGREES = 200.0;
    
    /** Low-pass filter alpha for turret angle smoothing. 
     *  Lower = smoother but more lag, Higher = more responsive but more jitter.
     *  Range: (0, 1]. Recommended: 0.15-0.25 */
    private static final double TURRET_ANGLE_FILTER_ALPHA = 0.2;

    // ==================== HARDWARE CONSTANTS ====================
    // gearing
    public static final double TURRET_GEAR_TEETH = 84.0; // number of teeth on fixed turret gear
    public static final double PINION_ENCODER_TEETH = 10.0; // number of teeth on pinion gear (driving turret)
    public static final double FOLLOWER_ENCODER_TEETH = 13.0; // number of teeth on follower gear
    public static final double TURRET_GEAR_RATIO = TURRET_GEAR_TEETH / PINION_ENCODER_TEETH; // gear ratio from motor to turret

    public static final double DEGREES_PER_REV = 360.0; // degrees in one revolution
    public static final double NORMALIZED_REVOLUTION = 1.0; // one full revolution in normalized units

    public static final double ENCODER_COMBINED_TEETH = PINION_ENCODER_TEETH * FOLLOWER_ENCODER_TEETH;
    public static final double ENCODER_COMBINED_PERIOD_REV = ENCODER_COMBINED_TEETH / PINION_ENCODER_TEETH;
    public static final double ENCODER_COMBINED_PERIOD_TURRET_REV =
        ENCODER_COMBINED_PERIOD_REV * (PINION_ENCODER_TEETH / TURRET_GEAR_TEETH);


    public static final double TURRET_CENTER_OFFSET_DEG = -88.1; // subtracted from robot relative heading
    public static final double TURRET_ROBOT_OFFSET_DEG = 47.8; // subtracted from robot relative heading to get turret relative heading

    public static final Translation2d TURRET_OFFSET_FROM_CENTER = new Translation2d(-0.3, -0.2); // distance from the center of the robot to the center of the turret, in meters 

    // ==================== INSTANCE VARIABLES ====================
    
    private boolean overrideAutomaticAiming = false;
    private TurretIO turretIO;
    
    /** Low-pass filter to smooth target angle and reduce jitter from vision noise */
    private final LowPassFilter angleFilter = new LowPassFilter(TURRET_ANGLE_FILTER_ALPHA);

    public Turret() {
        super("Turret", new TurretIOInputsAutoLogged());
    }

    @Override
    public void onPeriodic() {
        if (this.overrideAutomaticAiming) {
            this.turretIO.setTurretAngleRobotRelativeDegrees(0);
            this.turretIO.setTurretFeedforward(0);
        } else {
            double rawAngle = getTurretAngleDegreesFieldRelative();
            double filteredAngle = filterAngleWithWrapping(rawAngle);
            
            // Get feedforward from moving shot compensation
            double turretFeedforward = Targeting.getTurretFeedforward();
            
            Logger.recordOutput("Turret/RawTargetAngleDeg", rawAngle);
            Logger.recordOutput("Turret/FilteredTargetAngleDeg", filteredAngle);
            Logger.recordOutput("Turret/MovingShotFeedforwardDegPerSec", turretFeedforward);
            
            this.turretIO.setTurretAngleFieldRelativeDegrees(filteredAngle);
            this.turretIO.setTurretFeedforward(turretFeedforward);
        }
    }
    
    /**
     * Applies low-pass filtering to the angle while handling angle wrapping correctly.
     * This prevents issues when the angle crosses the +/-180 boundary.
     */
    private double filterAngleWithWrapping(double rawAngle) {
        if (!angleFilter.isInitialized()) {
            angleFilter.reset(rawAngle);
            return rawAngle;
        }
        
        double currentFiltered = angleFilter.get();
        
        // Handle angle wrapping - find the shortest path
        double delta = rawAngle - currentFiltered;
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
        
        // Apply filter to the delta, then add back to current
        double adjustedRaw = currentFiltered + delta;
        double filtered = angleFilter.calculate(adjustedRaw);
        
        // Wrap result back to [-180, 180]
        if (filtered > 180) {
            filtered -= 360;
            angleFilter.reset(filtered);
        } else if (filtered < -180) {
            filtered += 360;
            angleFilter.reset(filtered);
        }
        
        return filtered;
    }

    public double getTurretAngleDegreesFieldRelative() {
        Translation2d toGoal = Targeting.differenceBetweenRobotAndTarget();

        double angleToTarget = toGoal.getAngle().getDegrees();
        Logger.recordOutput("Targeting/AngleToTargetDeg", angleToTarget);
        return angleToTarget;
    }


    @Override
    protected Runnable setupDataRefresher() {
        turretIO = new TurretIOTalonFX(RobotContainer.getDrive());
        return useAsyncDataRefresher(turretIO);
    }

    public void toggleAimingOverride() {
        RobotContainer.getLEDController().switchPreset("fixed");
        this.overrideAutomaticAiming = !this.overrideAutomaticAiming;
    }

    /**
     * Checks if the turret is at the target angle, within the tolerance defined by TURRET_AIMING_TOLERANCE_DEGREES.
     * @return true if the turret is at the target angle, false otherwise
     */
    
    @AutoLogOutput(key="Turret/IsAtTargetAngle")
    public boolean isAtTargetAngle() {
        return Math.abs(io.turretAngularVelocityDegreesPerSecond) < 200.0;
    }

    public void changeTrim(double deltaDegrees) {
        turretIO.changeTurretTrim(deltaDegrees);
    }
}
