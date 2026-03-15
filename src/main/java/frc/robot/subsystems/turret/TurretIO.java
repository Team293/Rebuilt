package frc.robot.subsystems.turret;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends IORefresher, BaseIO<TurretIO.TurretIOInputs> {

    @AutoLog
    public static class TurretIOInputs extends BaseInputClass {
        public double fieldRelativeTurretAngleDegrees = 0.0; // measured turret heading on the field, in degrees [-180, 180)
        public double turretRelativeTurretAngleDegrees = 0.0; // measured turret heading in turret-frame (after center-offset), in degrees [-180, 180)
        public double robotRelativeTurretAngleDegrees = 0.0; // measured turret heading in robot-frame (after robot-offset), in degrees [-180, 180)
        public double targetFieldRelativeTurretDegrees = 0.0; // raw field-relative target sent to the turret, in degrees
        public double targetRobotRelativeTurretDegrees = 0.0; // robot-relative target after wrapping and trim applied, in degrees
        public double targetTurretMotorRotations = 0.0; // setpoint sent to the TalonFX position controller, in motor rotations
        public double turretOffsetRotations = 0.0; // motor position written at last zero/recalibration, in rotations
        public double turretMotorPositionRotations = 0.0; // raw TalonFX rotor position, in rotations
        public double pinionEncoderRotations = 0.0; // raw absolute position of the pinion (drive) CANcoder, in rotations [0, 1)
        public double followerEncoderRotations = 0.0; // raw absolute position of the follower CANcoder, in rotations [0, 1)
        public double rawTurretMechanismRotations = 0.0; // continuous turret mechanism position from CRT unwrapping, in revolutions
        public double pinionEncoderRevsCalculated = 0.0; // continuous pinion position resolved by the CRT algorithm, in revolutions
        public double turretTrimDegrees = 0.0; // operator-applied fine-trim offset, in degrees
        public double targetTurretDegreesFilteredFieldRelative = 0.0; // filtered field-relative target angle sent to the turret, in degrees [-180, 180)
        public double turretAngleDegreesRobotRelativeFiltered = 0.0; // filtered measured turret angle in robot-frame, in degrees [-180, 180)
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

    /**
     * Changes the turret trim by a certain amount of degrees. This is used to make minor adjustments to the turret angle based on operator controller input.
     * @param deltaDegrees the amount of degrees to change the turret trim by, in degrees. Positive values will adjust the turret angle clockwise, and negative values will adjust the turret angle counterclockwise.
     */
    void changeTurretTrim(double deltaDegrees);
}
