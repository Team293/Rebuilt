package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterIOTalonFXTest {
    @Test
    void risingRpmTargetGetsExtraKickOnlyWhenWellBelowTarget() {
        assertEquals(2, ShooterIOTalonFX.selectFlywheelSlot(50.1, 50.0, 47.0, false));
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(50.1, 50.0, 48.5, false));
        assertEquals(1, ShooterIOTalonFX.selectFlywheelSlot(50.1, 50.0, 48.5, true));
        assertEquals(2, ShooterIOTalonFX.selectFlywheelSlot(55.0, 50.0, 50.0, true));
    }

    @Test
    void stoppingRetreatImmediatelyRestoresOriginalGainEvenWhileBelowTarget() {
        assertEquals(2, ShooterIOTalonFX.selectFlywheelSlot(55.0, 50.0, 51.0, false));
        assertEquals(1, ShooterIOTalonFX.selectFlywheelSlot(55.0, 55.0, 51.0, true));
        assertEquals(1, ShooterIOTalonFX.selectFlywheelSlot(55.1, 55.0, 52.5, true));
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(50.0, 55.1, 54.0, true));
    }

    @Test
    void stationaryHoldingShotRecoveryAndInitialSpinupKeepExistingGains() {
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(50.0, 50.0, 49.0, false));
        assertEquals(1, ShooterIOTalonFX.selectFlywheelSlot(50.0, 50.0, 46.0, false));
        assertEquals(1, ShooterIOTalonFX.selectFlywheelSlot(50.0, 0.0, 0.0, false));
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(50.01, 50.0, 48.0, false));
    }

    @Test
    void extraKickReleasesNearTargetAndForFallingOrStoppedCommands() {
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(50.1, 50.0, 48.6, true));
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(50.1, 50.0, 51.0, true));
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(45.0, 50.0, 50.0, true));
        assertEquals(0, ShooterIOTalonFX.selectFlywheelSlot(0.0, 50.0, 45.0, true));
        assertEquals(1, ShooterIOTalonFX.selectFlywheelSlot(50.1, 50.1, 48.0, true));
    }

    @Test
    void risingTargetAndShotSpeedDropsEngageRecovery() {
        assertTrue(ShooterIOTalonFX.shouldRecoverFlywheel(50.0, 0.0, false));
        assertTrue(ShooterIOTalonFX.shouldRecoverFlywheel(55.0, 50.0, false));
        assertTrue(ShooterIOTalonFX.shouldRecoverFlywheel(50.0, 46.0, false));
    }

    @Test
    void recoveryReleasesNearTargetAndDoesNotBoostOverspeed() {
        assertFalse(ShooterIOTalonFX.shouldRecoverFlywheel(50.0, 48.5, true));
        assertFalse(ShooterIOTalonFX.shouldRecoverFlywheel(50.0, 50.0, true));
        assertFalse(ShooterIOTalonFX.shouldRecoverFlywheel(45.0, 50.0, true));
        assertFalse(ShooterIOTalonFX.shouldRecoverFlywheel(0.0, 0.0, true));
    }

    @Test
    void recoveryHysteresisAvoidsChatterDuringSpeedHolding() {
        boolean recovering = false;
        double[] measured = {47.1, 46.9, 47.1, 48.0, 48.4, 48.5, 48.0, 47.1};
        boolean[] expected = {false, true, true, true, true, false, false, false};
        for (int i = 0; i < measured.length; i++) {
            recovering = ShooterIOTalonFX.shouldRecoverFlywheel(50.0, measured[i], recovering);
            assertEquals(expected[i], recovering, "sample " + i);
        }
    }

    @Test
    void hoodTravelEndpointsAndMidpointMatchDegrees() {
        assertEquals(0.0, ShooterIOTalonFX.angleToEncoder(15.0), 1e-12);
        assertEquals(0.5, ShooterIOTalonFX.angleToEncoder(30.0), 1e-12);
        assertEquals(1.0, ShooterIOTalonFX.angleToEncoder(45.0), 1e-12);
        assertEquals(15.0, ShooterIOTalonFX.encoderToAngle(0.0), 1e-12);
        assertEquals(30.0, ShooterIOTalonFX.encoderToAngle(0.5), 1e-12);
        assertEquals(45.0, ShooterIOTalonFX.encoderToAngle(1.0), 1e-12);
    }

    @Test
    void commandedAnglesMatchDisplayedAnglesAcrossTravel() {
        for (double degrees = 15.0; degrees <= 45.0; degrees += 0.25) {
            assertEquals(degrees,
                    ShooterIOTalonFX.encoderToAngle(ShooterIOTalonFX.angleToEncoder(degrees)), 1e-12);
        }
    }

    @Test
    void outOfRangeCommandsClampToTravelLimits() {
        assertEquals(0.0, ShooterIOTalonFX.angleToEncoder(0.0), 1e-12);
        assertEquals(1.0, ShooterIOTalonFX.angleToEncoder(90.0), 1e-12);
    }
}
