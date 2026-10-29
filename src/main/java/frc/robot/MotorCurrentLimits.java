package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

/** Starting limits in amps; validate against installed motors, breakers, and on-robot logs. */
public final class MotorCurrentLimits {
    private MotorCurrentLimits() {}

    public static final double DRIVE_STATOR_AMPS = 80.0;

    public static CurrentLimitsConfigs drive() { return limits(DRIVE_STATOR_AMPS, 60, 40, 0.5); }
    public static CurrentLimitsConfigs steer() { return limits(40, 25, 25, 0); }
    public static CurrentLimitsConfigs intake() { return limits(60, 40, 25, 0.5); }
    public static CurrentLimitsConfigs findexer() { return limits(40, 40, 30, 0.5); }
    public static CurrentLimitsConfigs trigger() { return limits(30, 25, 25, 0); }
    public static CurrentLimitsConfigs turret() { return limits(40, 25, 25, 0); }
    public static CurrentLimitsConfigs hood() { return limits(10, 10, 10, 0); }
    public static CurrentLimitsConfigs flywheel() { return limits(100, 60, 40, 1.0); }

    // Supply is capped at peak immediately. After actively limiting for burstSeconds,
    // Phoenix lowers the cap to sustained until current drops below sustained.
    // Equal peak/sustained values give a fixed cap with no burst allowance.
    // Return fresh configs because Phoenix configuration objects are mutable.
    private static CurrentLimitsConfigs limits(
            double stator, double peak, double sustained, double burstSeconds) {
        return new CurrentLimitsConfigs()
                .withStatorCurrentLimit(stator)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(peak)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(sustained)
                .withSupplyCurrentLowerTime(burstSeconds);
    }
}
