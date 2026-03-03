package frc.robot.subsystems.turret.calc;

/**
 * Utility class for computing absolute turret position using two geared absolute encoders
 * with a CRT-like search approach.
 *
 * Hardware setup (example):
 * - Turret gear: 140 teeth
 * - Encoder A: e.g. 13-tooth gear driving an absolute encoder (reads 0-1 rev)
 * - Encoder B: e.g. 11-tooth gear driving an absolute encoder (reads 0-1 rev)
 *
 * The combination gives unique positions over (encoderA_teeth x encoderB_teeth) teeth.
 * Beyond that range, continuity tracking (unwrapping) is used.
 */
public class TurretMath {

    // Hardware tooth counts
    private static final double TURRET_GEAR_TEETH         = 140.0;
    private static final double ENCODER_A_TEETH           = 13.0;
    private static final double ENCODER_B_TEETH           = 11.0;

    // Derived encoder combination values
    private static final double ENCODER_COMBINED_TEETH    = ENCODER_A_TEETH * ENCODER_B_TEETH;
    private static final double ENCODER_COMBINED_PERIOD_REV = ENCODER_COMBINED_TEETH / TURRET_GEAR_TEETH;  
    private static final double HALF_ENCODER_COMBINED_PERIOD_REV = ENCODER_COMBINED_PERIOD_REV / 2.0;

    private static final double NORMALIZED_REV = 1.0;                // encoder reading range (0..1)
    private static final double HALF_NORMALIZED_REV = NORMALIZED_REV / 2.0;
    private static final double DEGREES_PER_REV = 360.0;
    private static final double RAD_PER_REV = 2.0 * Math.PI;
    private static final double MOTOR_UNITS_PER_REV = 14.0;          // scale used in degreesToMotorPosition

    // Defaults / initial values
    private static final double DEFAULT_OFFSET_DEGREES = 0.0;
    private static final double DEFAULT_LAST_POSITION_REVS = 0.0;

    // State
    private static double offsetDegrees = DEFAULT_OFFSET_DEGREES;   // Calibration offset (degrees)
    private static double lastPositionRevs = DEFAULT_LAST_POSITION_REVS; // Unwrapped continuous position
    private static boolean initialized = false;

    /**
     * Helper method to perform modulus that always returns a positive result, wrapping x into [0, m).
     * @param x value to wrap
     * @param m modulus (e.g. 1.0 for normalized encoder readings)
     * @return wrapped value in [0, m)
     */
    private static double mod(double x, double m) {
        return ((x % m) + m) % m;
    }

    /**
     * Computes the raw absolute turret position in revolutions of the encoder-A gear
     * (i.e. position in "encoder-A-gear-equivalent revolutions").
     * Range is approximately 0 to (combined_teeth / encoderA_teeth) revolutions of the encoder-A gear.
     *
     * This uses a search-based approach across the possible integer wraps of the encoder-A reading
     * to find the branch that best matches the encoder-B reading.
     *
     * @param encoderAReading  Absolute encoder A reading (normalized [0, 1))
     * @param encoderBReading  Absolute encoder B reading (normalized [0, 1))
     * @return       Raw turret position in encoder-A-gear revolutions
     */
    public static double getRawPositionPrimaryRevs(double encoderAReading, double encoderBReading) {
        double offsetRevs = offsetDegrees / DEGREES_PER_REV;

        // Apply offset and wrap to [0,1)
        encoderAReading = mod(encoderAReading - offsetRevs, NORMALIZED_REV);
        encoderBReading = mod(encoderBReading - offsetRevs, NORMALIZED_REV);

        double bestError = Double.POSITIVE_INFINITY;
        double bestPosition = 0.0;

        // Search over the possible integer wraps of the secondary/primary relationship
        int searchCount = (int) ENCODER_B_TEETH; // number of distinct branches to check (encoder B teeth)
        for (int k = 0; k < searchCount; k++) {
            double assumedPrimaryRevs = encoderAReading + k;
            // Predict what the encoder-B reading should be if this is the correct branch
            double predictedEncoderB = mod(assumedPrimaryRevs * (ENCODER_A_TEETH / ENCODER_B_TEETH), NORMALIZED_REV);

            double error = Math.abs(predictedEncoderB - encoderBReading);
            // Handle wrap-around distance
            if (error > HALF_NORMALIZED_REV) {
                error = NORMALIZED_REV - error;
            }

            if (error < bestError) {
                bestError = error;
                bestPosition = assumedPrimaryRevs;
            }
        }

        return bestPosition;
    }

    /**
     * Returns the turret angle in revolutions, unwrapped for continuous multi-turn motion.
     * Applies offset and continuity tracking.
     *
     * @param encoderAReading   Absolute encoder A reading [0,1)
     * @param encoderBReading   Absolute encoder B reading [0,1)
     * @return       Continuous turret position in revolutions (can be >1 or <0)
     */
    public static double getTurretAngleRevs(double encoderAReading, double encoderBReading) {
        double rawPrimaryRevs = getRawPositionPrimaryRevs(encoderAReading, encoderBReading);
        double rawTurretRevs = rawPrimaryRevs * (ENCODER_A_TEETH / TURRET_GEAR_TEETH);

        // Apply offset again (in turret space)
        rawTurretRevs -= offsetDegrees / DEGREES_PER_REV;

        // Wrap raw reading into one encoded period for comparison
        double wrapped = mod(rawTurretRevs, ENCODER_COMBINED_PERIOD_REV);

        if (!initialized) {
            lastPositionRevs = wrapped;
            initialized = true;
            return wrapped;
        }

        // Compute shortest path delta (assuming small motion between calls)
        double delta = wrapped - lastPositionRevs;

        // Unwrap using the known encoder combined period
        if (delta > HALF_ENCODER_COMBINED_PERIOD_REV) {
            delta -= ENCODER_COMBINED_PERIOD_REV;
        } else if (delta < -HALF_ENCODER_COMBINED_PERIOD_REV) {
            delta += ENCODER_COMBINED_PERIOD_REV;
        }

        lastPositionRevs += delta;
        return lastPositionRevs;
    }

    /**
     * Convert turret revolutions to degrees, wrapped to [0, 360).
     * Use this when you only care about single-turn angle.
     */
    public static double toDegreesWrapped(double turretRevs) {
        return mod(turretRevs, NORMALIZED_REV) * DEGREES_PER_REV;
    }

    /**
     * Convert turret revolutions to radians, wrapped to [0, 2pi).
     */
    public static double toRadiansWrapped(double turretRevs) {
        return mod(turretRevs, NORMALIZED_REV) * RAD_PER_REV;
    }

    public static double degreesToMotorPosition(double turretDegrees) {
        return (turretDegrees * MOTOR_UNITS_PER_REV) / DEGREES_PER_REV;
    }

    public static double normalizeTurretHeading(double turretHeading, double zeroDegrees) {
        double newHeading = turretHeading - zeroDegrees;

        if (newHeading < 0) {
            newHeading = DEGREES_PER_REV - Math.abs(newHeading);
        }

        return newHeading;
    }

    /**
     * Convert turret revolutions to total accumulated degrees (can be >360 or <0).
     * Use this when you want continuous angle for PID or motion profiling.
     */
    public static double toDegreesContinuous(double turretRevs) {
        return turretRevs * DEGREES_PER_REV;
    }

    // Calibration / zeroing
    public static void setOffsetDegrees(double degrees) {
        offsetDegrees = degrees;
    }

    public static double getOffsetDegrees() {
        return offsetDegrees;
    }

    // Reset continuity tracker
    public static void resetContinuity() {
        initialized = false;
        lastPositionRevs = DEFAULT_LAST_POSITION_REVS;
    }

    public static double toRad(double angle) {
        return angle * (Math.PI / DEGREES_PER_REV);
    }
}
