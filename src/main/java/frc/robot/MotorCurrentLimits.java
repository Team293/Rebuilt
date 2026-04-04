package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.measure.Current;

/**
 * Centralized current limits for all motors on the robot.
 * These limits help prevent brownouts and protect motors from damage.
 */
public enum MotorCurrentLimits {
    /** Drive motors - moderate limit for efficient driving */
    DRIVE_MOTORS(30),
    
    /** Turn/steer motors - lower limit since azimuth requires less torque */
    TURN_MOTORS(20),
    
    /** Turret rotation motor */
    TURRET(30),
    
    /** Intake roller motor */
    INTAKE(30),
    
    /** Trigger motor */
    TRIGGER(30),
    
    /** Findexer motor */
    FINDEXER(30),
    
    /** Flywheel motor - higher limit for fast spin-up */
    FLYWHEEL(40);

    private final int statorCurrentLimitAmps;

    MotorCurrentLimits(int statorCurrentLimitAmps) {
        this.statorCurrentLimitAmps = statorCurrentLimitAmps;
    }

    /**
     * Get the stator current limit in amps.
     * @return stator current limit in amps
     */
    public int getAmps() {
        return statorCurrentLimitAmps;
    }

    /**
     * Get the stator current limit as a WPILib Current measure.
     * @return stator current limit as a Current measure
     */
    public Current getCurrent() {
        return Amps.of(statorCurrentLimitAmps);
    }

    /**
     * Create a CurrentLimitsConfigs object with this current limit enabled.
     * @return configured CurrentLimitsConfigs with stator current limit enabled
     */
    public CurrentLimitsConfigs toCurrentLimitsConfigs() {
        return new CurrentLimitsConfigs()
                .withStatorCurrentLimit(getCurrent())
                .withStatorCurrentLimitEnable(true);
    }
}
