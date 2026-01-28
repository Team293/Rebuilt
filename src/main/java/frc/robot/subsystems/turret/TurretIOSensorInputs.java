package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;
import frc.robot.subsystems.vision.photon.Camera;

public class TurretIOSensorInputs implements TurretIO, IORefresher {
    private final CANcoder turretEncoder;
    private final Camera turretCamera;
    private final TalonFX turretMotor;

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0.0);

    private static final double kTurretGearRatio = 100.0; // 100:1, this isn't the real value

    private static final double kTurretMinAngleDegrees = -180.0;
    private static final double kTurretMaxAngleDegrees = 180.0;

    public TurretIOSensorInputs(Camera camera) {
        this.turretEncoder = new CANcoder(CanID.TURRET_ENCODER.getID());
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());
        this.turretCamera = camera;
    }

    public void setTurretAngle(double angle) {
        double clamped = Math.max(kTurretMinAngleDegrees, Math.min(kTurretMaxAngleDegrees, angle));
        double motorRotations = (clamped / 360.0) * kTurretGearRatio;

        turretMotor.setControl(mmRequest.withPosition(motorRotations));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAngleDegrees  = turretEncoder.getPosition().getValue().in(Units.Degree);
    }

    @Override
    public void refreshData() {

    }
}
