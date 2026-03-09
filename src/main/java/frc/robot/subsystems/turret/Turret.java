package frc.robot.subsystems.turret;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.targeting.ShotCompensation;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    public static final double TURRET_AIMING_TOLERANCE_DEGREES = 2.0; // degrees within which we consider the turret to be aimed at the target (+-)

    // HARDWARE CONSTANTS
    // gearing
    public static final double TURRET_GEAR_TEETH = 84.0; // number of teeth on fixed turret gear
    public static final double PINION_ENCODER_TEETH = 10.0; // number of teeth on pinion gear (driving turret)
    public static final double FOLLOWER_ENCODER_TEETH = 13.0; // number of teeth on follower gear
    public static final double TURRET_GEAR_RATIO = TURRET_GEAR_TEETH / PINION_ENCODER_TEETH; // gear ratio from motor to turret

    public static final double DEGREES_PER_REV = 360.0; // degrees in one revolution
    public static final double NORMALIZED_REVOLUTION = 1.0; // one full revolution in normalized units

    public static final double ENCODER_COMBINED_TEETH = PINION_ENCODER_TEETH * FOLLOWER_ENCODER_TEETH;
    public static final double ENCODER_COMBINED_PERIOD_REV = ENCODER_COMBINED_TEETH / PINION_ENCODER_TEETH;

    public static final double PINION_ENCODER_OFFSET = 0.0; // offset for pinion encoder in degrees, to be determined by calibration
    public static final double FOLLOWER_ENCODER_OFFSET = 0.0; // offset for follower encoder in degrees, to be determined by calibration

    private TurretIO turretIO;

    public Turret() {
        super("Turret", new TurretIOInputsAutoLogged());
    }

    @Override
    public void onPeriodic() {
        ShotCompensation.AdjustedShot shotData = Targeting.getShotData();

        if (shotData != null) {
            double newTargetAngleDeg = shotData.turretAngleDeg();

            this.turretIO.setTurretAngleFieldRelativeDegrees(newTargetAngleDeg);
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        turretIO = new TurretIOTalonFX(RobotContainer.getDrive());
        return useAsyncDataRefresher(turretIO);
    }

    /**
     * Checks if the turret is at the target angle, within the tolerance defined by TURRET_AIMING_TOLERANCE_DEGREES.
     * @return true if the turret is at the target angle, false otherwise
     */
    public boolean isAtTargetAngle() {
        double minAngle = io.turretAngleDegrees - TURRET_AIMING_TOLERANCE_DEGREES;
        double maxAngle = io.turretAngleDegrees + TURRET_AIMING_TOLERANCE_DEGREES;

        return io.targetTurretMotorRotations >= minAngle && io.targetTurretMotorRotations <= maxAngle;
    }
}
