package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterTest {
    @Test
    void stationaryAtSpeedAndExistingUnderspeedAllowanceRemainReady() {
        assertTrue(Shooter.isFlywheelReady(50.0, 50.0));
        assertTrue(Shooter.isFlywheelReady(50.0, 46.0));
        assertFalse(Shooter.isFlywheelReady(50.0, 45.0));
    }

    @Test
    void stoppingRetreatPausesFeedingUntilWheelMatchesLowerCommand() {
        assertTrue(Shooter.isFlywheelReady(55.0, 55.0));
        assertFalse(Shooter.isFlywheelReady(50.0, 55.0));
        assertFalse(Shooter.isFlywheelReady(50.0, 51.5));
        assertTrue(Shooter.isFlywheelReady(50.0, 51.0));
    }

    @Test
    void stoppedAndInvalidSpeedsCannotFeed() {
        assertFalse(Shooter.isFlywheelReady(0.0, 0.0));
        assertFalse(Shooter.isFlywheelReady(0.0, 50.0));
        assertFalse(Shooter.isFlywheelReady(50.0, Double.NaN));
        assertFalse(Shooter.isFlywheelReady(Double.NaN, 50.0));
    }
}
