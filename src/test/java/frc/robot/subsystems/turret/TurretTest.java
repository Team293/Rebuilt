package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TurretTest {
    @Test
    void warningRampsUpTowardEitherSideOfWrap() {
        for (double direction : new double[] {-1.0, 1.0}) {
            assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(direction * 0.0), 1e-9);
            assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(direction * 134.0), 1e-9);
            assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(direction * 135.0), 1e-9);
            assertEquals(direction * 0.5, Turret.calculateDirectionalWraparoundRumble(direction * 157.5), 1e-9);
            assertEquals(direction * (44.0 / 45.0),
                    Turret.calculateDirectionalWraparoundRumble(direction * 179.0), 1e-9);
        }
    }

    @Test
    void warningSwitchesSidesAcrossWrapAndClearsOutsideRange() {
        assertEquals(Turret.calculateDirectionalWraparoundRumble(179.0),
                -Turret.calculateDirectionalWraparoundRumble(181.0), 1e-9);
        assertEquals(Turret.calculateDirectionalWraparoundRumble(-179.0),
                -Turret.calculateDirectionalWraparoundRumble(-181.0), 1e-9);
        assertEquals(1.0, Math.abs(Turret.calculateDirectionalWraparoundRumble(180.0)), 1e-9);
        assertEquals(1.0, Math.abs(Turret.calculateDirectionalWraparoundRumble(-180.0)), 1e-9);
        assertEquals(-0.5, Turret.calculateDirectionalWraparoundRumble(202.5), 1e-9);
        assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(225.0), 1e-9);
        assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(360.0), 1e-9);
    }

    @Test
    void invalidTargetDoesNotRumble() {
        assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(Double.NaN));
        assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(Double.POSITIVE_INFINITY));
        assertEquals(0.0, Turret.calculateDirectionalWraparoundRumble(Double.NEGATIVE_INFINITY));
    }
}
