package frc.robot.subsystems.findexer;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class FindexerJamRecoveryTest {
    private final FindexerJamRecovery recovery = new FindexerJamRecovery();

    private double feed(double time, double rps, double amps) {
        return recovery.calculate(time, true, false, true, rps, amps);
    }

    /** Starts feeding into a jam, confirms it, and returns the reversal start time. */
    private double jam(double start) {
        assertEquals(-120.0, feed(start, 0.0, 40.0));
        assertEquals(-120.0, feed(start + 0.51, 0.0, 40.0));
        assertEquals(30.0, feed(start + 0.77, 0.0, 40.0));
        return start + 0.77;
    }

    @Test
    void allowsStartupAndIgnoresBriefBallImpacts() {
        assertEquals(-120.0, feed(0.0, 0.0, 40.0));
        assertEquals(-120.0, feed(0.49, 0.0, 40.0));
        assertEquals(-120.0, feed(0.6, -60.0, 8.0));
        assertEquals(-120.0, feed(1.0, 0.0, 40.0));
        assertEquals(-120.0, feed(1.20, 0.0, 40.0));
        assertEquals(-120.0, feed(1.22, -60.0, 8.0));
        // A second impact must start its own confirmation timer.
        assertEquals(-120.0, feed(1.3, 0.0, 40.0));
        assertEquals(-120.0, feed(1.5, 0.0, 40.0));
        assertEquals(0, recovery.getAttempts());
    }

    @Test
    void requiresBothSlowForwardMotionAndHighCurrent() {
        feed(0.0, -60.0, 8.0);
        for (double time : new double[] {0.6, 1.0, 2.0}) {
            assertEquals(-120.0, feed(time, 0.0, 10.0));
        }
        for (double time : new double[] {2.1, 2.5, 3.0}) {
            assertEquals(-120.0, feed(time, -50.0, 40.0));
        }
        assertEquals(0, recovery.getAttempts());
    }

    @Test
    void reversesBrieflyThenRetriesWithAnotherStartupAllowance() {
        double reverse = jam(0.0);
        assertEquals(1, recovery.getAttempts());
        assertEquals(30.0, feed(reverse + 0.19, 10.0, 40.0));
        assertEquals(-120.0, feed(reverse + 0.21, 10.0, 40.0));
        assertEquals(-120.0, feed(reverse + 0.70, 0.0, 40.0));
        assertEquals(FindexerJamRecovery.State.FEEDING, recovery.getState());
    }

    @Test
    void wrongDirectionDoesNotCountAsSuccessfulFeeding() {
        feed(0.0, 60.0, 40.0);
        feed(0.6, 60.0, 40.0);
        assertEquals(30.0, feed(0.86, 60.0, 40.0));
    }

    @Test
    void stoppingOrDisablingCancelsAReversalImmediately() {
        double reverse = jam(0.0);
        assertEquals(0.0, recovery.calculate(reverse + 0.05, false, false, true, 0.0, 40.0));
        assertEquals(FindexerJamRecovery.State.STOPPED, recovery.getState());
        assertEquals(0, recovery.getAttempts());
        assertEquals(-120.0, feed(reverse + 0.10, 0.0, 40.0));
    }

    @Test
    void manualReverseOverridesRecoveryEvenWithoutAFeedRequestOrTelemetry() {
        double reverse = jam(0.0);
        assertEquals(120.0, recovery.calculate(reverse + 0.05, false, true, false, 0.0, 0.0));
        assertEquals(FindexerJamRecovery.State.MANUAL_REVERSE, recovery.getState());
        assertEquals(0, recovery.getAttempts());
        // Releasing manual reverse restarts the forward startup timer.
        assertEquals(-120.0, feed(2.0, 0.0, 40.0));
        assertEquals(-120.0, feed(2.4, 0.0, 40.0));
    }

    @Test
    void invalidTelemetryStopsRecoveryAndDoesNotExtendAStallTimer() {
        feed(0.0, 0.0, 40.0);
        feed(0.6, 0.0, 40.0);
        assertEquals(0.0, recovery.calculate(0.8, true, false, false, 0.0, 40.0));
        assertEquals(-120.0, feed(1.0, 0.0, 40.0));
        assertEquals(-120.0, feed(1.4, 0.0, 40.0));
        assertEquals(0, recovery.getAttempts());

        feed(1.6, 0.0, 40.0);
        assertEquals(30.0, feed(1.9, 0.0, 40.0));
        assertEquals(0.0, recovery.calculate(1.95, true, false, false, 0.0, 40.0));
        assertEquals(FindexerJamRecovery.State.STOPPED, recovery.getState());
        assertEquals(1, recovery.getAttempts());
    }

    @Test
    void nonFiniteMeasurementsCannotTriggerClearing() {
        assertEquals(0.0, feed(0.0, Double.NaN, 40.0));
        assertEquals(0.0, feed(1.0, 0.0, Double.NaN));
        assertEquals(0, recovery.getAttempts());
    }

    @Test
    void persistentJamStopsAfterThreeReversalsUntilRequestIsReleased() {
        jam(0.0);
        jam(1.0);
        double reverse = jam(2.0);
        assertEquals(3, recovery.getAttempts());
        assertEquals(-120.0, feed(reverse + 0.21, 0.0, 40.0));
        feed(reverse + 0.72, 0.0, 40.0);
        assertEquals(0.0, feed(reverse + 0.98, 0.0, 40.0));
        assertEquals(FindexerJamRecovery.State.LOCKED_OUT, recovery.getState());
        assertEquals(0.0, feed(10.0, -60.0, 8.0));
        // A telemetry dropout must not replenish attempts or remove the lockout.
        recovery.calculate(10.1, true, false, false, 0.0, 0.0);
        assertEquals(0.0, feed(10.2, 0.0, 40.0));
        recovery.calculate(11.0, false, false, true, 0.0, 0.0);
        assertEquals(-120.0, feed(12.0, 0.0, 40.0));
        assertEquals(0, recovery.getAttempts());
    }

    @Test
    void sustainedHealthyForwardMotionReplenishesAttempts() {
        jam(0.0);
        feed(1.0, -60.0, 8.0);
        feed(1.9, -60.0, 8.0);
        assertEquals(1, recovery.getAttempts());
        // One slow sample interrupts the healthy-motion timer.
        feed(1.95, 0.0, 10.0);
        feed(2.0, -60.0, 8.0);
        feed(2.9, -60.0, 8.0);
        assertEquals(1, recovery.getAttempts());
        feed(3.01, -60.0, 8.0);
        assertEquals(0, recovery.getAttempts());
    }
}
