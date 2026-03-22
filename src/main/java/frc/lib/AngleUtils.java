package frc.lib;

import edu.wpi.first.math.MathUtil;

public class AngleUtils {

    /**
     * Filters an angle value using a low-pass filter, handling angle wrapping around a specified range.
     * This ensures the filter takes the shortest path across the wrap boundary (e.g., -180 to 180 degrees).
     *
     * @param filter the LowPassFilter instance to use
     * @param rawAngleDeg the new raw angle value in degrees
     * @param minDeg the minimum angle of the wrap range (e.g., -180.0)
     * @param maxDeg the maximum angle of the wrap range (e.g., 180.0)
     * @return the filtered angle value, wrapped back into [minDeg, maxDeg)
     */
    public static double filterWrappedAngleDeg(LowPassFilter filter, double rawAngleDeg, double minDeg, double maxDeg) {
        if (!filter.isInitialized()) {
            filter.reset(rawAngleDeg);
            return rawAngleDeg;
        }

        // Unwrap the new angle around the previous filtered value so the filter always takes the short path
        double previousFilteredAngleDeg = filter.get();
        double unwrappedAngleDeg = previousFilteredAngleDeg
                + MathUtil.inputModulus(rawAngleDeg - previousFilteredAngleDeg, minDeg, maxDeg);
        double filteredUnwrappedAngleDeg = filter.calculate(unwrappedAngleDeg);
        return MathUtil.inputModulus(filteredUnwrappedAngleDeg, minDeg, maxDeg);
    }
}
