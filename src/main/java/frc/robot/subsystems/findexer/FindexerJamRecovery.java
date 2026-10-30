package frc.robot.subsystems.findexer;

/** Timed jam clearing, independent of hardware so ball impacts and retries can be tested. */
final class FindexerJamRecovery {
    // Motor-shaft RPS, matching the existing findexer velocity request.
    static final double FEEDING_RPS = -120.0;
    static final double CLEARING_RPS = 30.0;

    // Initial tuning values: distinguish a sustained near-stall from a normal ball impact.
    static final double STALL_SPEED_RPS = 15.0;
    static final double STALL_CURRENT_AMPS = 30.0;
    static final double STARTUP_SECONDS = 0.50;
    static final double STALL_SECONDS = 0.25;
    static final double REVERSE_SECONDS = 0.20;
    static final double HEALTHY_SPEED_RPS = 45.0;
    static final double HEALTHY_RESET_SECONDS = 1.0;
    static final int MAX_ATTEMPTS = 3;

    enum State { STOPPED, FEEDING, REVERSING, MANUAL_REVERSE, LOCKED_OUT }

    private State state = State.STOPPED;
    private int attempts = 0;
    private double feedStartedAt = Double.NaN;
    private double stallStartedAt = Double.NaN;
    private double reverseStartedAt = Double.NaN;
    private double healthyStartedAt = Double.NaN;

    double calculate(double now, boolean feedingRequested, boolean manualReverse,
            boolean telemetryValid, double motorRps, double statorCurrentAmps) {
        // The caller suppresses both requests while disabled. Manual reverse takes priority.
        if (manualReverse || !feedingRequested) {
            resetTiming();
            attempts = 0;
            state = manualReverse ? State.MANUAL_REVERSE : State.STOPPED;
            return manualReverse ? -FEEDING_RPS : 0.0;
        }

        if (!telemetryValid || !Double.isFinite(motorRps) || !Double.isFinite(statorCurrentAmps)) {
            resetTiming();
            if (state != State.LOCKED_OUT) state = State.STOPPED;
            return 0.0;
        }
        if (state == State.LOCKED_OUT) return 0.0;

        if (state == State.REVERSING) {
            if (now - reverseStartedAt < REVERSE_SECONDS) return CLEARING_RPS;
            state = State.STOPPED; // Give every forward retry a fresh startup allowance.
        }
        if (state != State.FEEDING) {
            resetTiming();
            feedStartedAt = now;
            state = State.FEEDING;
        }

        double forwardRps = motorRps * Math.signum(FEEDING_RPS);
        // Only sustained, successful forward motion restores the retry budget.
        if (forwardRps >= HEALTHY_SPEED_RPS) {
            if (Double.isNaN(healthyStartedAt)) healthyStartedAt = now;
            if (now - healthyStartedAt >= HEALTHY_RESET_SECONDS) attempts = 0;
        } else {
            healthyStartedAt = Double.NaN;
        }

        boolean stalled = forwardRps < STALL_SPEED_RPS
                && Math.abs(statorCurrentAmps) >= STALL_CURRENT_AMPS;
        if (now - feedStartedAt < STARTUP_SECONDS || !stalled) {
            stallStartedAt = Double.NaN;
        } else {
            if (Double.isNaN(stallStartedAt)) stallStartedAt = now;
            if (now - stallStartedAt >= STALL_SECONDS) {
                if (attempts >= MAX_ATTEMPTS) {
                    state = State.LOCKED_OUT;
                    return 0.0;
                }
                attempts++;
                reverseStartedAt = now;
                state = State.REVERSING;
                return CLEARING_RPS;
            }
        }
        return FEEDING_RPS;
    }

    State getState() {
        return state;
    }

    int getAttempts() {
        return attempts;
    }

    private void resetTiming() {
        feedStartedAt = Double.NaN;
        stallStartedAt = Double.NaN;
        reverseStartedAt = Double.NaN;
        healthyStartedAt = Double.NaN;
    }
}
