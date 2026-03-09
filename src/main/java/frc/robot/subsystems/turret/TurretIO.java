package frc.robot.subsystems.turret;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends IORefresher, BaseIO<TurretIO.TurretIOInputs> {

    @AutoLog
    public static class TurretIOInputs extends BaseInputClass {
        public double turretAngleDegrees = 0.0; // current angle of the turret, field-relative, in degrees
        public double targetTurretMotorRotations = 0.0; // target position for the turret motor, in rotations
        public double normalizedTurretMotorRotations = 0.0; // calculated turret motor rotations, updated when recalculation is called
    }

    /**
     * Set the angle of the turret, field-relative, in degrees. 0 degrees is facing straight forward, positive angles are clockwise, and negative angles are counterclockwise. [-180, 180]
     * @param fieldRelativeAngleDegrees field-relative angle to set the turret to, in degrees. 0 degrees is facing straight forward, positive angles are clockwise, and negative angles are counterclockwise. [-180, 180]
     */
    void setTurretAngleFieldRelativeDegrees(double fieldRelativeAngleDegrees);

    /**
     * Recalculates the turret motor zero position to fix any encoder drift.
     */
    void recalculateTurretMotorZeroPosition();
}
