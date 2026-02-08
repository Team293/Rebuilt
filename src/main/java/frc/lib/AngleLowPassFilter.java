package frc.lib;

public class AngleLowPassFilter {

    private double filteredX;
    private double filteredY;
    private boolean initialized;

    private double alpha; // smoothing factor (0..1)

    /**
     * @param alpha smoothing factor.
     *              smaller = smoother but more lag.
     *              recommended starting range: 0.1 - 0.2
     */
    public AngleLowPassFilter(double alpha) {
        setAlpha(alpha);
        this.initialized = false;
    }

    /**
     * Update filter with new angle measurement.
     *
     * @param angleDeg angle in degrees
     * @return filtered angle in degrees
     */
    public double calculate(double angleDeg) {
        double rad = Math.toRadians(angleDeg);

        double x = Math.cos(rad);
        double y = Math.sin(rad);

        if (!initialized) {
            filteredX = x;
            filteredY = y;
            initialized = true;
        }

        filteredX += alpha * (x - filteredX);
        filteredY += alpha * (y - filteredY);

        return Math.toDegrees(Math.atan2(filteredY, filteredX));
    }

    /**
     * Reset filter state to a known angle.
     */
    public void reset(double angleDeg) {
        double rad = Math.toRadians(angleDeg);
        filteredX = Math.cos(rad);
        filteredY = Math.sin(rad);
        initialized = true;
    }

    /**
     * Change smoothing strength dynamically.
     */
    public void setAlpha(double alpha) {
        if (alpha <= 0 || alpha > 1) {
            throw new IllegalArgumentException("Alpha must be in (0, 1]");
        }
        this.alpha = alpha;
    }

    /**
     * Whether filter has received first value.
     */
    public boolean isInitialized() {
        return initialized;
    }
}
