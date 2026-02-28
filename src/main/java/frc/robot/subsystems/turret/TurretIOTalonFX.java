package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.CanID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.calc.TurretMath;

import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;
    private final CommandSwerveDrivetrain drive;

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> encoderSignal;
    private final BaseStatusSignal pinionEncoderSignal;
    private final BaseStatusSignal followerEncoderSignal;

    private static final double kTurretGearRatio = 140/10; // turret ring: 140 teeth, motor pinion: 10 teeth

    private final double turretRotationOffset; // offset in motor rotations, calculated at startup, units in motor rotations
    private final double turretDegreesOffset = 246.7; // offset in degrees to align turret with front of robot

    private double targetAngleDeg = 0.0; // target angle of the turret in degrees

    public TurretIOTalonFX(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicAcceleration = 60; // rot/sec^2
        mm.MotionMagicCruiseVelocity = 30; // rot/sec

        Slot0Configs config = new Slot0Configs();
        config.kP = 1;
        config.kI = 0.01;
        config.kD = 0.3;
        config.kS = 0.194;
        config.kV = 0.1167;

        this.turretMotor.getConfigurator().apply(config);
        this.turretMotor.getConfigurator().apply(mm);

        CANcoder pinionEncoder = new CANcoder(CanID.TURRET_PINION_CANCODER.getID());
        CANcoder followerEncoder = new CANcoder(CanID.TURRET_FOLLOWER_CANCODER.getID());

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        // constrain sensor readings to [0, 1)
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);

        pinionEncoder.getConfigurator().apply(encoderConfig);
        followerEncoder.getConfigurator().apply(encoderConfig);

        this.pinionEncoderSignal = pinionEncoder.getAbsolutePosition();
        this.followerEncoderSignal = followerEncoder.getAbsolutePosition();

        this.encoderSignal = this.turretMotor.getPosition();

        double pinionEncoderValue = this.pinionEncoderSignal.getValueAsDouble();
        double followerEncoderValue = this.followerEncoderSignal.getValueAsDouble();

        // set offset of the turret on startup
        double turretRotations = TurretMath.getTurretAngleRevs(pinionEncoderValue, followerEncoderValue);
    
        double turretDegrees = TurretMath.normalizeTurretHeading(
            TurretMath.toDegreesWrapped(turretRotations),
            this.turretDegreesOffset
        );

        double currentMotorRevs = this.turretMotor.getPosition().getValueAsDouble();
        double toZeroRevs = TurretMath.degreesToMotorPosition(turretDegrees);

        this.turretRotationOffset = currentMotorRevs + toZeroRevs;
    }

    @Override
    public void setTurretAngle(double fieldTargetHeadingDeg) {
        this.targetAngleDeg = fieldTargetHeadingDeg;
        fieldTargetHeadingDeg = MathUtil.inputModulus(fieldTargetHeadingDeg, 0.0, 360.0);

        // convert robot heading to [0, 360) range
        double robotHeadingDeg =
            MathUtil.inputModulus(
                drive.getPose().getRotation().getDegrees(),
                0.0, 360.0
            );

        // dt = time we expect turret to reach commanded angle, tunable
        double dt = 0.025;
        double predictedHeadingDeg =
            robotHeadingDeg + this.drive.getRobotOmegaDegPerSec() * dt;

        // turret angle in (-180, 180] range, where positive is counterclockwise relative to the robot's forward direction, and negative is clockwise
        double turretAngleDeg =
            MathUtil.inputModulus(
                fieldTargetHeadingDeg - predictedHeadingDeg,
                -180.0, 180.0
            );

        // convert turret angle to motor rotations, accounting for gear ratio and offset
        double rotations = (turretAngleDeg / 360.0) * kTurretGearRatio;
        rotations = -rotations + this.turretRotationOffset;

        // calculate feedforward to counteract robot rotation, using a simple linear model with gain determined empirically
        double motorRps =
            -(this.drive.getRobotOmegaDegPerSec() / 360.0) * kTurretGearRatio;

        // set the motor to the desired position with feedforward to counteract robot rotation
        this.turretMotor.setControl(
            this.mmVoltage
                .withPosition(rotations)
                    // 0.1167 is an empirically determined gain to convert from motor RPS to voltage needed to hold position against rotation
                .withFeedForward(motorRps * 0.1167)
        );
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.pinionEncoder = this.pinionEncoderSignal.getValueAsDouble();
        inputs.followerEncoder = this.followerEncoderSignal.getValueAsDouble();
        inputs.turretSetPointDegrees = this.targetAngleDeg;

        double turretRotations = TurretMath.getTurretAngleRevs(inputs.pinionEncoder, inputs.followerEncoder);

        inputs.robotOmegaDegPerSec = this.drive.getRobotOmegaDegPerSec();

        // convert raw encoder readings to turret angle in degrees, accounting for gear ratio and offset
        inputs.turretAngleDegrees = TurretMath.normalizeTurretHeading(
            TurretMath.toDegreesWrapped(turretRotations),
            turretDegreesOffset
        );

        // creates a position for the turret based on robot position, rotated by turret angle for logging
        inputs.turretPosition = new Pose2d(drive.getPose().getTranslation(), new Rotation2d(TurretMath.toRad(inputs.turretAngleDegrees)));
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(encoderSignal, pinionEncoderSignal, followerEncoderSignal);
    }
}
