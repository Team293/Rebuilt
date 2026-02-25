package frc.lib;

public class LowPassFilter {

    private double filteredValue;
    private boolean initialized;
    private double alpha; // smoothing factor (0..1)

    /**
     * @param alpha smoothing factor.
     *              smaller = smoother but more lag.
     *              recommended starting range: 0.1 - 0.2
     */
    public LowPassFilter(double alpha) {
        setAlpha(alpha);
        this.initialized = false;
    }

    /**
     * Update filter with new measurement.
     *
     * @param input new raw value
     * @return filtered value
     */
    public double calculate(double input) {

        if (!initialized) {
            filteredValue = input;
            initialized = true;
            return filteredValue;
        }

        filteredValue += alpha * (input - filteredValue);

        return filteredValue;
    }

    /**
     * Reset filter to a known value.
     */
    public void reset(double value) {
        filteredValue = value;
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

    public boolean isInitialized() {
        return initialized;
    }

    public double get() {
        return filteredValue;
    }
}