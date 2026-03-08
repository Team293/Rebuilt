package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CanID;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.calc.TurretMath;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;
    private final CommandSwerveDrivetrain drive;

    private static final double TURRET_LIMIT_DEG = 160.0;

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> encoderSignal;
    private final BaseStatusSignal pinionEncoderSignal;
    private final BaseStatusSignal followerEncoderSignal;

    private static final double kTurretGearRatio = 84.0/10.0; // turret ring: 84 teeth, motor pinion: 10 teeth

    private final double turretDegreesOffset = 180.0; // offset in degrees to align turret with front of robot

    private double targetAngleDeg = 0.0; // target angle of the turret in degrees

    public TurretIOTalonFX(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicAcceleration = 10; // rot/sec^2
        mm.MotionMagicCruiseVelocity = 10; // rot/sec

        // Motor configuration
        Slot0Configs config = getTurretMotorConfig();

        this.turretMotor.getConfigurator().apply(config);
        this.turretMotor.getConfigurator().apply(mm);

        CANcoder pinionEncoder = new CANcoder(CanID.TURRET_PINION_CANCODER.getID());
        CANcoder followerEncoder = new CANcoder(CanID.TURRET_FOLLOWER_CANCODER.getID());

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

        turretMotor.setPosition(-toZeroRevs);
    }

    /**
     * Set the turret angle to a target field angle
     * @param fieldTargetHeadingDeg field relative angle to set turret to 
     */
    @Override
    public void setTurretAngle(double fieldTargetHeadingDeg) {
        Logger.recordOutput("Turret/TargetAngle", fieldTargetHeadingDeg);
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

        if (turretAngleDeg > TURRET_LIMIT_DEG) {
            turretAngleDeg -= 360.0;
        } else if (turretAngleDeg < -TURRET_LIMIT_DEG) {
            turretAngleDeg += 360.0;
        }

        // convert turret angle to motor rotations, accounting for gear ratio and offset
        double turretAngleForMotor = turretAngleDeg;
        if (turretAngleForMotor > 180.0) {
            turretAngleForMotor -= 360.0;
        }
        Logger.recordOutput("Turret/TargetAngleForMotor", turretAngleForMotor);
        double rotations = -(turretAngleForMotor / 360.0) * kTurretGearRatio;

        Logger.recordOutput("Turret/TargetRotations", rotations);
        // set the motor to the desired position with feedforward to counteract robot rotation
        this.turretMotor.setControl(
            this.mmVoltage
                .withPosition(rotations)
                    // 0.1167 is an empirically determined gain to convert from motor RPS to voltage needed to hold position against rotation
                // .withFeedForward(motorRps * 0.1167)
        );
    }
    
    
    /**
     * Periodically called to update the Turret information for logging
     * @param inputs TurretIOInputs object to update
     */
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.pinionEncoder = this.pinionEncoderSignal.getValueAsDouble();
        inputs.followerEncoder = this.followerEncoderSignal.getValueAsDouble();
        inputs.turretSetPointDegrees = this.targetAngleDeg;

        double turretRotations = TurretMath.getTurretAngleRevs(inputs.pinionEncoder, inputs.followerEncoder);

        inputs.robotOmegaDegPerSec = this.drive.getRobotOmegaDegPerSec();

        // convert raw encoder readings to turret angle in degrees, accounting for gear ratio and offset
        double turretAngleDegreesNonNormalized = TurretMath.normalizeTurretHeading(
            TurretMath.toDegreesWrapped(turretRotations),
            turretDegreesOffset
        );

        // normalize to -180 to 180 range
        if (turretAngleDegreesNonNormalized > 180.0) {
            inputs.turretAngleDegrees = turretAngleDegreesNonNormalized - 360.0;
        } else if (turretAngleDegreesNonNormalized < -180.0) {
            inputs.turretAngleDegrees = turretAngleDegreesNonNormalized + 360.0;
        } else {
            inputs.turretAngleDegrees = turretAngleDegreesNonNormalized;
        }

        // creates a position for the turret based on robot position, rotated by turret angle for logging
        inputs.turretPosition = new Pose2d(drive.getPose().getTranslation(), new Rotation2d(TurretMath.toRad(inputs.turretAngleDegrees)));
    }

    /**
     * Periodically refreshes encoder signal
     * @note This is called automatically
     */
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(encoderSignal, pinionEncoderSignal, followerEncoderSignal);
    }

    public static Slot0Configs getTurretMotorConfig() {
        Slot0Configs config = new Slot0Configs();
    
        // config.kP = 1;
        // config.kI = 0.0;
        // config.kD = 0.0;

        // config.kS = 0.25;
        // config.kV = 0.20;

        config.kP = 0.0;
        config.kI = 0.0;
        config.kD = 0.0;

        config.kS = 0.0;
        config.kV = 0.0;

        return config;
    }
}
