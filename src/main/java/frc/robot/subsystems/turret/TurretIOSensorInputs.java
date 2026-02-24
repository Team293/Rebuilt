package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.calc.TurretMath;
import frc.robot.subsystems.vision.photon.Camera;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class TurretIOSensorInputs implements TurretIO, IORefresher {
    private final TalonFX turretMotor;
    private final DutyCycleEncoder pinionEncoder;
    private final DutyCycleEncoder followerEncoder;

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> encoderSignal;

    private static final double kTurretGearRatio = 140/10;
    private final double turretRotationOffset;
    private final double turretDegreesOffset = 246.7;

    public TurretIOSensorInputs() {
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicAcceleration = 60;
        mm.MotionMagicCruiseVelocity = 30;
        mm.MotionMagicJerk = 0;

        this.turretMotor.getConfigurator().apply(mm);
        this.pinionEncoder = new DutyCycleEncoder(0);
        this.followerEncoder = new DutyCycleEncoder(1);

        this.encoderSignal = turretMotor.getPosition();

        double pinionEncoderValue = pinionEncoder.get();
        double followerEncoderValue = followerEncoder.get();

        // set offset of the turret on startup
        double turretRotations = TurretMath.getTurretAngleRevs(pinionEncoderValue, followerEncoderValue);
    
        double turretDegrees = TurretMath.normalizeTurretHeading(
            TurretMath.toDegreesWrapped(turretRotations),
            turretDegreesOffset
        );

        Logger.recordOutput("Turret/InitialHeading", turretDegrees);

        double currentMotorRevs = turretMotor.getPosition().getValueAsDouble();
        double toZeroRevs = TurretMath.degreesToMotorPosition(turretDegrees);

        turretRotationOffset = currentMotorRevs + toZeroRevs;

        Logger.recordOutput("Turret/ToZeroRevs", toZeroRevs);
        Logger.recordOutput("Turret/TurretRotationOffset", turretRotationOffset);
    }   

    public void setTurretAngle(double fieldAngleDeg) {
        fieldAngleDeg = MathUtil.inputModulus(fieldAngleDeg, 0.0, 360.0);

        var drive = RobotContainer.getDrive();

        double robotHeadingDeg =
            MathUtil.inputModulus(
                drive.getPose().getRotation().getDegrees(),
                0.0, 360.0
            );

        double robotOmegaDegPerSec =
            drive.getState().Speeds.omegaRadiansPerSecond
                * 180.0 / Math.PI;

        double dt = 0.025;
        double predictedHeadingDeg =
            robotHeadingDeg + robotOmegaDegPerSec * dt;

        double turretAngleDeg =
            MathUtil.inputModulus(
                fieldAngleDeg + predictedHeadingDeg,
                -180.0, 180.0
            );

        double rotations = (turretAngleDeg / 360.0) * kTurretGearRatio;
        rotations = -rotations + turretRotationOffset;

        double motorRps =
            -(robotOmegaDegPerSec / 360.0) * kTurretGearRatio;

        turretMotor.setControl(
            mmVoltage
                .withPosition(rotations)
                .withFeedForward(motorRps * 0.1167)
        );

        Logger.recordOutput("Turret/CommandedRotations", rotations);

        Logger.recordOutput("Turret/RobotOmegaDegPerSec", robotOmegaDegPerSec);
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.pinionEncoder = this.pinionEncoder.get();
        inputs.followerEncoder = this.followerEncoder.get();
        double turretRotations = TurretMath.getTurretAngleRevs(inputs.pinionEncoder, inputs.followerEncoder);
    
        inputs.turretAngleDegrees = TurretMath.normalizeTurretHeading(
            TurretMath.toDegreesWrapped(turretRotations),
            turretDegreesOffset
        );
        
        Logger.recordOutput("Turret/CRTRevs", turretRotations);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(encoderSignal);
    }

    @Override
    public double getAngleOffsetFromPose(
            Translation2d robotPose,
            Translation2d target
    ) {
        double fieldAngleDeg = Math.toDegrees(
            Math.atan2(
                target.getY() - robotPose.getY(),
                target.getX() - robotPose.getX()
            )
        );
        return MathUtil.inputModulus(-fieldAngleDeg, -180, 180);
    }
}
