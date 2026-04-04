package frc.robot.subsystems.turret;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends IORefresher, BaseIO<TurretIO.TurretIOInputs> {

    @AutoLog
    public static class TurretIOInputs extends BaseInputClass {
        public double turretAngleDegreesFieldRelative = 0.0; // current angle of the turret, field-relative, in degrees
        public double turretAngleDegreesTurretRelative = 0.0; // current angle of the turret, turret-relative, in degrees, after setting a turret center offset
        public double turretAngleDegreesRobotRelative = 0.0; // current angle of the turret, robot-relative, in degrees, after setting a robot center offset
        public double targetTurretDegrees = 0.0; // target angle for the turret, field-relative, in degrees
        public double processedTargetTurretDegrees = 0.0; // processed target angle for the turret, field-relative, in degrees, updated when recalculation is called
        public double targetTurretMotorRotations = 0.0; // target position for the turret motor, in rotations
        public double turretOffsetRotations = 0.0; // calculated turret motor rotations, updated when recalculation is called
        public double turretMotorPositionRotations = 0.0; // current position of the turret motor, in rotations
        public double pinionEncoderRotations = 0.0; // current rotations of the pinion encoder
        public double followerEncoderRotations = 0.0; // current rotations of the follower
        public double rawTurretMechanismRotations = 0.0; // raw rotations of the entire turret mechanism
        public double turretTrimDegrees = 0.0; // minor adjustment to the turret angle based on operator controller input, in degrees
        public double turretAngularVelocityDegreesPerSecond = 0.0; // current angular velocity of the turret, in degrees per second
    }

    /**
     * Set the angle of the turret, field-relative, in degrees. 0 degrees is facing straight forward, positive angles are clockwise, and negative angles are counterclockwise. [-180, 180]
     * @param fieldRelativeAngleDegrees field-relative angle to set the turret to, in degrees. 0 degrees is facing straight forward, positive angles are clockwise, and negative angles are counterclockwise. [-180, 180]
     */
    void setTurretAngleFieldRelativeDegrees(double fieldRelativeAngleDegrees);

    /**
     * Set the angle of the turret relative to the robot.
     */
    void setTurretAngleRobotRelativeDegrees(double robotRelativeDegrees);

    /**
     * Recalculates the turret motor zero position to fix any encoder drift.
     */
    void recalculateTurretMotorZeroPosition();

    /**
     * Changes the turret trim by a certain amount of degrees. This is used to make minor adjustments to the turret angle based on operator controller input.
     * @param deltaDegrees the amount of degrees to change the turret trim by, in degrees. Positive values will adjust the turret angle clockwise, and negative values will adjust the turret angle counterclockwise.
     */
    void changeTurretTrim(double deltaDegrees);

    /**
     * Set the angular velocity feedforward for the turret to compensate for robot rotation during moving shots.
     * @param feedforwardDegPerSec feedforward angular velocity in degrees per second
     */
    default void setTurretFeedforward(double feedforwardDegPerSec) {
        // Default implementation does nothing - override in TurretIOTalonFX
    }
}
