package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;

/**
 * Utility class for computing absolute turret position using two geared absolute encoders
 * with the Chinese Remainder Theorem (CRT) approach.
 * 
 * Hardware setup:
 * - Turret gear: 140 teeth
 * - Encoder 13: 13-tooth gear driving an absolute encoder (reads 0–1 rev)
 * - Encoder 11: 11-tooth gear driving an absolute encoder (reads 0–1 rev)
 * 
 * The combination gives unique positions over 11 × 13 = 143 teeth → 143/140 ≈ 1.0214 turret revolutions.
 * Beyond that range, continuity tracking (unwrapping) is used.
 */
public class TurretMath {

    private static final double GEAR_TEETH_TURRET = 140.0;
    private static final double GEAR_TEETH_ENC13  = 13.0;
    private static final double GEAR_TEETH_ENC11  = 11.0;
    private static final double PERIOD_TEETH      = GEAR_TEETH_ENC11 * GEAR_TEETH_ENC13; // 143
    private static final double PERIOD_REV        = PERIOD_TEETH / GEAR_TEETH_TURRET;    // ≈1.0214

    private static double offsetDegrees = 0;           // Calibration offset (degrees)
    private static double lastPositionRevs = 0.0;        // Unwrapped continuous position
    private static boolean initialized = false;

    private static double mod(double x, double m) {
        return ((x % m) + m) % m;
    }

    /**
     * Computes the raw absolute turret position in revolutions of the 13-tooth gear
     * (i.e. position in "13-gear equivalent revolutions").
     * Range is approximately 0 to (143/13) ≈ 11.0 revolutions of the 13-gear.
     * 
     * This uses a search-based CRT solution (most reliable for real hardware).
     *
     * @param enc13  Absolute encoder on 13-tooth gear, normalized [0, 1)
     * @param enc11  Absolute encoder on 11-tooth gear, normalized [0, 1)
     * @return       Raw turret position in 13-gear revolutions (multiply by 13/140 for turret revs)
     */
    public static double getRawPosition13Revs(double enc13, double enc11) {
        double offsetRevs = offsetDegrees / 360.0;

        // Apply offset and wrap to [0,1)
        enc13 = mod(enc13 - offsetRevs, 1.0);
        enc11 = mod(enc11 - offsetRevs, 1.0);

        double bestError = Double.POSITIVE_INFINITY;
        double bestPosition = 0.0;

        // Search over the 11 possible integer steps of the slower gear
        for (int k = 0; k < (int) GEAR_TEETH_ENC11; k++) {
            double assumed13Revs = enc13 + k;
            // Predict what enc11 *should* read if this is the correct branch
            double predicted11 = mod(assumed13Revs * (GEAR_TEETH_ENC13 / GEAR_TEETH_ENC11), 1.0);

            double error = Math.abs(predicted11 - enc11);
            // Handle wrap-around distance
            if (error > 0.5) {
                error = 1.0 - error;
            }

            if (error < bestError) {
                bestError = error;
                bestPosition = assumed13Revs;
            }
        }

        // You can add a sanity check here if desired
        // if (bestError > 0.015) { /* signal invalid reading */ }

        return bestPosition;
    }

    /**
     * Returns the turret angle in revolutions, unwrapped for continuous multi-turn motion.
     * Applies offset and continuity tracking.
     * 
     * @param enc13  Absolute encoder on 13-tooth gear [0,1)
     * @param enc11  Absolute encoder on 11-tooth gear [0,1)
     * @return       Continuous turret position in revolutions (can be >1 or <0)
     */
    public static double getTurretAngleRevs(double enc13, double enc11) {
        double raw13Revs = getRawPosition13Revs(enc13, enc11);
        double rawTurretRevs = raw13Revs * (GEAR_TEETH_ENC13 / GEAR_TEETH_TURRET);

        // Apply offset again (in turret space)
        rawTurretRevs -= offsetDegrees / 360.0;

        // Wrap raw reading into one period for comparison
        double wrapped = mod(rawTurretRevs, PERIOD_REV);

        if (!initialized) {
            lastPositionRevs = wrapped;
            initialized = true;
            return wrapped;
        }

        // Compute shortest path delta (assuming small motion between calls)
        double delta = wrapped - lastPositionRevs;

        // Unwrap using the known period
        if (delta > PERIOD_REV / 2.0) {
            delta -= PERIOD_REV;
        } else if (delta < -PERIOD_REV / 2.0) {
            delta += PERIOD_REV;
        }

        lastPositionRevs += delta;
        return lastPositionRevs;
    }

    /**
     * Convert turret revolutions to degrees, wrapped to [0, 360).
     * Use this when you only care about single-turn angle.
     */
    public static double toDegreesWrapped(double turretRevs) {
        return mod(turretRevs, 1.0) * 360.0;
    }

    /**
     * Convert turret revolutions to radians, wrapped to [0, 2π).
     */
    public static double toRadiansWrapped(double turretRevs) {
        return mod(turretRevs, 1.0) * 2.0 * Math.PI;
    }

    /**
     * Convert turret revolutions to total accumulated degrees (can be >360 or <0).
     * Use this when you want continuous angle for PID or motion profiling.
     */
    public static double toDegreesContinuous(double turretRevs) {
        return turretRevs * 360.0;
    }

    // Calibration / zeroing
    public static void setOffsetDegrees(double degrees) {
        offsetDegrees = degrees;
    }

    public static double getOffsetDegrees() {
        return offsetDegrees;
    }

    // Reset continuity tracker (call on robot enable or after large jumps)
    public static void resetContinuity() {
        initialized = false;
        lastPositionRevs = 0.0;
    }

    public static double toRad(double angle) {
        return angle * (Math.PI / 180);
    }
}