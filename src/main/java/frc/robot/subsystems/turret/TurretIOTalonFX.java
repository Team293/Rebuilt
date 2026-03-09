package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CanID;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TurretIOTalonFX implements TurretIO {
    // KS KV CONSTANTS
    private static final double kS = 0.25; // volts needed to overcome static friction
    private static final double kV = 0.20; // volts per (rotation per second) to maintain motion

    // SUBSYSTEMS
    private final CommandSwerveDrivetrain drive;

    // HARDWARE
    private final TalonFX turretMotor; // kraken x44
    private final CANcoder pinionEncoder; // wcp throughbore
    private final CANcoder followerEncoder; // wcp throughbore

    // SIGNALS
    private final StatusSignal<Angle> turretMotorPosition;
    private final StatusSignal<Angle> pinionEncoderSignal;
    private final StatusSignal<Angle> followerEncoderSignal;

    // CALIBRATION STATES (CRT)
    private boolean isInitialized = false; // whether the turret has been initialized with a known position yet
    private double lastPositionRevs = 0.0; // last calculated position of the turret in revolutions

    // VALUES
    private double targetTurretDegreesFieldRelative; // target angle of the turret in degrees, relative to the field
    private double processedTargetTurretDegreesFieldRelative; // processed target angle of the turret in degrees, relative to the field
    private double targetTurretAngleMotorRevs; // target angle of the turret in motor rotations
    private double calculatedMotorOffsetRevs; // calculated offset in motor rotations based on the current position of the turret and the pinion encoder reading

    // COMMANDS
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0.0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV); // ks, kv

    public TurretIOTalonFX(CommandSwerveDrivetrain drive) {
        // SUBSYSTEMS
        this.drive = drive;

        // HARDWARE
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());
        this.pinionEncoder = new CANcoder(CanID.TURRET_PINION_CANCODER.getID());
        this.followerEncoder = new CANcoder(CanID.TURRET_FOLLOWER_CANCODER.getID());

        // CONFIGURATIONS
        var turretMotorConfig = getTurretMotionConfigs();
        var pinionEncoderConfig = getPinionEncoderConfigs();
        var followerEncoderConfig = getFollowerEncoderConfigs();
        var turretMotorFeedbackConfig = getTurretMotorFeedbackConfigs();
        var turretSoftwareLimitConfig = getTurretSoftwareLimitConfigs();

        this.turretMotor.getConfigurator().apply(turretMotorConfig.getFirst());
        this.turretMotor.getConfigurator().apply(turretMotorConfig.getSecond());
        this.turretMotor.getConfigurator().apply(turretMotorFeedbackConfig);
        this.turretMotor.getConfigurator().apply(turretSoftwareLimitConfig);

        this.pinionEncoder.getConfigurator().apply(pinionEncoderConfig);
        this.followerEncoder.getConfigurator().apply(followerEncoderConfig);

        // SIGNALS
        this.turretMotorPosition = this.turretMotor.getPosition();
        this.pinionEncoderSignal = this.pinionEncoder.getAbsolutePosition();
        this.followerEncoderSignal = this.followerEncoder.getAbsolutePosition();

        // ZEROING POSITION
        recalculateTurretMotorZeroPosition();
    }

    @Override
    public void setTurretAngleFieldRelativeDegrees(double fieldRelativeAngleDegrees) {
        this.targetTurretDegreesFieldRelative = fieldRelativeAngleDegrees;
        double currentRobotHeading = this.drive.getPose().getRotation().getDegrees();

        // absolute robot-relative target, in motor rotations
        double targetRobotRelativeDeg = wrap180(fieldRelativeAngleDegrees - currentRobotHeading);
        double targetMotorRotations = targetRobotRelativeDeg / 180.0;

        this.processedTargetTurretDegreesFieldRelative = targetRobotRelativeDeg;
        this.targetTurretAngleMotorRevs = targetMotorRotations;

        this.turretMotor.setControl(
                mmRequest
                        .withPosition(targetMotorRotations)
                        .withFeedForward(calculateFeedforward())
        );
    }

    /**
     * @inheritDoc
     */
    @Override
    public void recalculateTurretMotorZeroPosition() {
        double currentTurretAngle = getTurretAngleRobotRelative(); // get the current angle of the turret in robot frame

        double normalizedPosition = currentTurretAngle / 180.0; // convert to normalized position [-1, 1]
        this.calculatedMotorOffsetRevs = normalizedPosition; // calculate the offset in motor rotations based on the current turret angle

        this.turretMotor.setPosition(normalizedPosition);
    }

    /**
     * Calculates the feedforward voltage to apply to the turret motor to counteract the rotation of the robot, based on the current angular velocity of the robot.
     * @return the feedforward value to apply to the turret rotation
     */
    private double calculateFeedforward() {
        // get the current angular velocity of the robot in radians per second
        double gyroOmegaRadPerSecond = drive.getState().Speeds.omegaRadiansPerSecond;

        double gyroOmegaDegPerSecond = gyroOmegaRadPerSecond * (180.0 / Math.PI); // convert to degrees per second

        return feedforward.calculate(gyroOmegaDegPerSecond);
    }

    @Override
    public void refreshData() {
        StatusSignal.refreshAll(this.turretMotorPosition, this.pinionEncoderSignal, this.followerEncoderSignal);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAngleDegrees = getTurretAngleFieldRelative();
        inputs.targetTurretMotorRotations = this.targetTurretAngleMotorRevs;
        inputs.normalizedTurretMotorRotations = this.calculatedMotorOffsetRevs;
        inputs.targetTurretDegrees = this.targetTurretDegreesFieldRelative;
        inputs.processedTargetTurretDegrees = this.processedTargetTurretDegreesFieldRelative;
        inputs.turretMotorPositionRotations = this.turretMotorPosition.getValueAsDouble();
        inputs.pinionEncoderRotations = this.pinionEncoderSignal.getValueAsDouble();
        inputs.followerEncoderRotations = this.followerEncoderSignal.getValueAsDouble();
    }

    // CONFIGURATIONS

    /**
     * Get the motor configurations for the turret motor.
     * @return the motor configurations for the turret motor
     */
    private Pair<Slot0Configs, MotionMagicConfigs> getTurretMotionConfigs() {
        Slot0Configs configs = new Slot0Configs();

        configs.kP = 1;
        configs.kI = 0.0;
        configs.kD = 0.0;

        configs.kS = kS;
        configs.kV = kV;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

        mmConfigs.MotionMagicAcceleration = 20; // rotations per second^2
        mmConfigs.MotionMagicCruiseVelocity = 10; // rotations per second

        return new Pair<>(configs, mmConfigs);
    }

    /**
     * Get the feedback configurations for the turret motor, which define the relationship between the motor rotations, the pinion encoder rotations, and the follower encoder rotations.
     * @return the feedback configurations for the turret motor
     */
    private FeedbackConfigs getTurretMotorFeedbackConfigs() {
        FeedbackConfigs configs = new FeedbackConfigs();

        // SensorToMechanismRatio = GR/2 means the motor completes 2 mechanism rotations per
        // full turret revolution, so 1 mechanism rotation = 180 degrees of turret travel.
        // Soft limits at +-1 mechanism rotation enforce the +-180 degrees physical range of the turret.
        configs.SensorToMechanismRatio = Turret.TURRET_GEAR_RATIO / 2.0;
        configs.RotorToSensorRatio = 1;

        return configs;
    }

    /**
     * Get the software limit switch configurations for the turret motor, which define the forward and reverse limits of the turret based on the motor position.
     * @return the software limit switch configurations for the turret motor
     */
    private SoftwareLimitSwitchConfigs getTurretSoftwareLimitConfigs() {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();

        configs.ForwardSoftLimitEnable = true;
        configs.ForwardSoftLimitThreshold = 1; // 1 rotation of the motor past the zero point

        configs.ReverseSoftLimitEnable = true;
        configs.ReverseSoftLimitThreshold = -1; // 1 rotation of the motor in the opposite direction past the zero point

        return configs;
    }

    /**
     * Get the encoder configurations for the turret encoders.
     * @return the encoder configurations for the turret encoders
     */
    public CANcoderConfiguration getEncoderConfigs() {
        CANcoderConfiguration configs = new CANcoderConfiguration();
        // constrain reading between [0, 1)
        configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        return configs;
    }

    public CANcoderConfiguration getPinionEncoderConfigs() {
        CANcoderConfiguration configs = getEncoderConfigs();
        configs.MagnetSensor.MagnetOffset = Turret.FOLLOWER_ENCODER_OFFSET;

        return configs;
    }

    public CANcoderConfiguration getFollowerEncoderConfigs() {
        CANcoderConfiguration configs = getEncoderConfigs();
        configs.MagnetSensor.MagnetOffset = Turret.PINION_ENCODER_OFFSET;

        return configs;
    }

    // CRT METHODS

    /**
     * Calculates the continuous position of the pinion (driving) encoder in revolutions
     * @return the continuous position of the pinion encoder in revolutions
     */
    private double getPinionEncoderRevs() {
        double pinionEncoderReading = positiveMod(this.pinionEncoderSignal.getValueAsDouble(), Turret.NORMALIZED_REVOLUTION);
        double followerEncoderReading = positiveMod(this.followerEncoderSignal.getValueAsDouble(), Turret.NORMALIZED_REVOLUTION);

        double bestError = Double.MAX_VALUE;
        double bestPosition = 0.0;

        int searchCount = (int) Turret.FOLLOWER_ENCODER_TEETH; // number of distinct branches to check (follower encoder teeth)
        for (int k = 0; k < searchCount; k++) {
            double assumedPinionRevs = pinionEncoderReading + k;
            double predictedFollowerReading = positiveMod(assumedPinionRevs * (Turret.PINION_ENCODER_TEETH / Turret.FOLLOWER_ENCODER_TEETH), Turret.NORMALIZED_REVOLUTION);
            double predictionError = Math.abs(predictedFollowerReading - followerEncoderReading);

            // check wrap-around error
            if (predictionError > Turret.NORMALIZED_REVOLUTION / 2.0) {
                predictionError = Turret.NORMALIZED_REVOLUTION - predictionError;
            }

            // if this branch has a better prediction error than the best one so far, update the best guess for the pinion encoder position
            if (predictionError < bestError) {
                bestError = predictionError;
                bestPosition = assumedPinionRevs;
            }
        }

        return bestPosition;
    }

    /**
     * Calculates the continuous position of  the turret in revolutions of the entire mechanism.
     * @return the continuous position of the turret in revolutions.
     */
    private double getTurretPositionRevs() {
        double rawPinionRevs = getPinionEncoderRevs();
        double rawTurretRevs = rawPinionRevs * (Turret.PINION_ENCODER_TEETH / Turret.TURRET_GEAR_TEETH); // convert pinion revolutions to turret revolutions

        double wrapped = positiveMod(rawTurretRevs, Turret.ENCODER_COMBINED_PERIOD_REV);

        if (!isInitialized) {
            this.lastPositionRevs = wrapped;
            isInitialized = true;
            return wrapped;
        }

        double delta = wrapped - this.lastPositionRevs;

        // if the change in position is greater than half the combined period, we have wrapped around the encoder, so we need to adjust the delta accordingly
        if (delta > Turret.ENCODER_COMBINED_PERIOD_REV / 2.0) {
            delta -= Turret.ENCODER_COMBINED_PERIOD_REV;
        } else if (delta < -Turret.ENCODER_COMBINED_PERIOD_REV / 2.0) {
            delta += Turret.ENCODER_COMBINED_PERIOD_REV;
        }

        lastPositionRevs += delta;
        return lastPositionRevs;
    }

    /**
     * Converts rotations of the turret mechanism to degrees (heading) of the turret.
     * @param turretRevs continuous revolutions of the turret
     * @return degrees of the turret from revolutions, continuous and unwrapped
     */
    private double revsToDegreesContinuous(double turretRevs) {
        return turretRevs * Turret.DEGREES_PER_REV;
    }

    /**
     * Gets the angle of the turret in robot space wrapped from [-180, 180)
     * @return the angle of the turret in robot space, wrapped
     */
    private double getTurretAngleRobotRelative() {
        double continuousRevs = getTurretPositionRevs();
        double turretAngleDegrees = revsToDegreesContinuous(continuousRevs);

        // apply offset and wrap to [-180, 180)
        return wrap180(turretAngleDegrees);
    }

    /**
     * Gets the angle of the turret in field space, wrapped from [-180, 180)
     * @return the angle of the turret in field space, wrapped
     */
    private double getTurretAngleFieldRelative() {
        double robotRelativeAngle = getTurretAngleRobotRelative();
        double currentRobotHeading = this.drive.getPose().getRotation().getDegrees();

        double fieldCentricContinuous = robotRelativeAngle - currentRobotHeading;

        return wrap180(fieldCentricContinuous);
    }

    // UTILITY METHODS

    /**
     * Wraps the input angle to be within the range [min, max).
     * @param input the angle to wrap
     * @param min the minimum angle of the range (inclusive)
     * @param max the maximum angle of the range (exclusive)
     * @return the wrapped angle within the range [min, max)
     */
    private double wrap(double input, double min, double max) {
        // input modulo the range size
        return MathUtil.inputModulus(
                input,
                min,
                max
        );
    }

    /**
     * Wraps the input angle to be within the range [-180, 180).
     * @param input the angle to wrap
     * @return the wrapped angle within the range [-180, 180)
     */
    private double wrap180(double input) {
        return wrap(input, -180.0, 180.0);
    }

    /**
     * A positive modulus function that wraps x into the range [0, m).
     * @param x the value to wrap
     * @param m the modulus
     * @return the wrapped value in the range [0, m)
     */
    private double positiveMod(double x, double m) {
        return ((x % m) + m) % m;
    }
}
