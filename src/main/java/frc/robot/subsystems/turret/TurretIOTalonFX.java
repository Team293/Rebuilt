package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.PositionVoltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.lib.LowPassFilter;
import frc.robot.CanID;
import frc.robot.MotorCurrentLimits;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TurretIOTalonFX implements TurretIO {
    /** Volts needed to overcome static friction */
    private static final double kS = 0.35;
    
    /** Volts per (rotation per second) to maintain motion */
    private static final double kV = 0.20;
    
    /** Proportional gain - increase for snappier response, decrease if oscillating */
    private static final double kP = 20.0;
    
    /** Integral gain - typically leave at 0 for position control */
    private static final double kI = 0.0;
    
    /** Derivative gain - increase to dampen oscillations */
    private static final double kD = 2.0;
    
    /** Static friction compensation for slot 0 */
    private static final double SLOT_kS = 0.4;
    
    /** Velocity feedforward for slot 0 */
    private static final double SLOT_kV = 0.4;

    // ==================== WRAP BOUNDARY HYSTERESIS ====================
    
    /** Hysteresis band in degrees to prevent oscillation at turret wrap boundaries.
     *  When within this many degrees of the limit, the turret will not wrap until
     *  it moves past limit + hysteresis in the opposite direction. */
    private static final double WRAP_HYSTERESIS_DEGREES = 15.0;
    // ==================== SUBSYSTEMS ====================
    private final CommandSwerveDrivetrain drive;

    // ==================== HARDWARE ====================
    private final TalonFX turretMotor; // kraken x44
    private final CANcoder pinionEncoder; // wcp throughbore
    private final CANcoder followerEncoder; // wcp throughbore

    // ==================== SIGNALS ====================
    private final StatusSignal<Angle> turretMotorPosition;
    private final StatusSignal<Angle> pinionEncoderSignal;
    private final StatusSignal<Angle> followerEncoderSignal;

    // ==================== CALIBRATION STATES (CRT) ====================
    private boolean isInitialized = false; // whether the turret has been initialized with a known position yet
    private double lastPositionRevs = 0.0; // last calculated position of the turret in revolutions
    private double lastPinionRevs = 0.0; // last calculated position of the pinion encoder in revolutions

    private double lastTurretAngleDegrees = 0.0; // last calculated angle of the turret in degrees, used for calculating angular velocity

    // ==================== VALUES ====================
    private double targetTurretDegreesFieldRelative; // target angle of the turret in degrees, relative to the field
    private double processedTargetTurretDegreesFieldRelative; // processed target angle of the turret in degrees, relative to the field
    private double targetTurretAngleMotorRevs; // target angle of the turret in motor rotations
    private double calculatedMotorOffsetRevs; // calculated offset in motor rotations based on the current position of the turret and the pinion encoder reading
    private double targetTurretDegreesTurretRelative = 0;

    // ==================== COMMANDS ====================
    private final PositionVoltage mmRequest = new PositionVoltage(0.0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV); // ks, kv

    private double turretTrimDegrees = 0.0;
    private double externalFeedforwardVoltage = 0.0; // feedforward voltage from moving shot compensation

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

        this.turretMotor.getConfigurator().apply(turretMotorConfig);
        this.turretMotor.getConfigurator().apply(turretMotorFeedbackConfig);
        this.turretMotor.getConfigurator().apply(turretSoftwareLimitConfig);
        this.turretMotor.getConfigurator().apply(MotorCurrentLimits.TURRET.toCurrentLimitsConfigs());
        // invert motor
        this.turretMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        this.pinionEncoder.getConfigurator().apply(pinionEncoderConfig);
        this.followerEncoder.getConfigurator().apply(followerEncoderConfig);

        // SIGNALS
        this.turretMotorPosition = this.turretMotor.getPosition();
        this.pinionEncoderSignal = this.pinionEncoder.getAbsolutePosition();
        this.followerEncoderSignal = this.followerEncoder.getAbsolutePosition();

        BaseStatusSignal.refreshAll(
            this.turretMotorPosition,
            this.pinionEncoderSignal,
            this.followerEncoderSignal
        );

        // ZEROING POSITION
        recalculateTurretMotorZeroPosition();
    }

    @Override
    public void setTurretAngleFieldRelativeDegrees(double fieldRelativeAngleDegrees) {
        this.targetTurretDegreesFieldRelative = fieldRelativeAngleDegrees;
        double currentRobotHeading = this.drive.getPose().getRotation().getDegrees();

        // Calculate robot-relative target angle
        double targetRobotRelativeDeg = fieldRelativeAngleDegrees - currentRobotHeading;
        
        // Normalize to [-180, 180] first for logging/display purposes
        this.processedTargetTurretDegreesFieldRelative = wrap180(targetRobotRelativeDeg);

        setTurretAngleRobotRelativeDegrees(targetRobotRelativeDeg);
    }

    public void setTurretAngleRobotRelativeDegrees(double robotRelativeAngleDegrees) {
        setTurretAngleTurretRelativeDegrees(robotRelativeAngleDegrees + Turret.TURRET_ROBOT_OFFSET_DEG);
    }

    private void setTurretAngleTurretRelativeDegrees(double angleDegrees) {
        this.targetTurretDegreesTurretRelative = angleDegrees;

        angleDegrees += turretTrimDegrees;
        
        // Get current turret position for hysteresis calculation
        double currentAngle = getTurretAngle();
        
        // Apply wrapping with hysteresis to prevent oscillation at boundaries
        angleDegrees = wrapToTurretRange(angleDegrees, currentAngle);
        
        // Final clamp to ensure we never exceed the physical limits
        angleDegrees = MathUtil.clamp(angleDegrees, 
            -Turret.TURRET_MAX_ANGLE_DEGREES, 
            Turret.TURRET_MAX_ANGLE_DEGREES);
        
        // Convert degrees to mechanism rotations (1 mechanism rotation = 180 degrees)
        double targetMotorRotations = angleDegrees / 180.0;
        this.targetTurretAngleMotorRevs = targetMotorRotations;
        
        
        mmRequest.Position = targetMotorRotations;
        mmRequest.FeedForward = externalFeedforwardVoltage; // Apply moving shot feedforward compensation
        this.turretMotor.setControl(
            mmRequest
        );
    }

    @Override
    public void setTurretFeedforward(double feedforwardDegPerSec) {
        // Convert deg/sec to mechanism rotations/sec (1 mechanism rotation = 180 degrees)
        double mechanismRotationsPerSec = feedforwardDegPerSec / 180.0;
        // Use the existing feedforward calculator to get voltage
        this.externalFeedforwardVoltage = feedforward.calculate(mechanismRotationsPerSec);
    }

    /**
     * @inheritDoc
     */
    @Override
    public void recalculateTurretMotorZeroPosition() {
        // absolute turret position from CRT
        this.calculatedMotorOffsetRevs = getTurretAngle() / 180.0;

        turretMotor.setPosition(this.calculatedMotorOffsetRevs);
    }

    /**
     * Calculates the feedforward voltage to apply to the turret motor to counteract the rotation of the robot, based on the current angular velocity of the robot.
     * @return the feedforward value to apply to the turret rotation
     */
    private double calculateFeedforward() {
        // get the current angular velocity of the robot in radians per second
        double gyroOmegaRadPerSecond = drive.getState().Speeds.omegaRadiansPerSecond;

        double mechanismRotationsPerSecond = gyroOmegaRadPerSecond / Math.PI;
        return feedforward.calculate(-mechanismRotationsPerSecond);
    }


    @Override
    public void refreshData() {
        StatusSignal.refreshAll(this.turretMotorPosition, this.pinionEncoderSignal, this.followerEncoderSignal);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAngleDegreesFieldRelative = getTurretAngleFieldRelative();
        inputs.turretAngleDegreesTurretRelative = getTurretAngle();
        inputs.turretAngleDegreesRobotRelative = getTurretAngleRobotRelative();
        inputs.targetTurretMotorRotations = this.targetTurretAngleMotorRevs;
        inputs.turretOffsetRotations = this.calculatedMotorOffsetRevs;
        inputs.targetTurretDegrees = this.targetTurretDegreesTurretRelative;
        inputs.processedTargetTurretDegrees = this.processedTargetTurretDegreesFieldRelative;
        inputs.turretMotorPositionRotations = this.turretMotorPosition.getValueAsDouble();
        inputs.pinionEncoderRotations = this.pinionEncoderSignal.getValueAsDouble();
        inputs.followerEncoderRotations = this.followerEncoderSignal.getValueAsDouble();
        inputs.rawTurretMechanismRotations = this.getTurretPositionRevs();
        inputs.turretTrimDegrees = turretTrimDegrees;
        inputs.turretAngularVelocityDegreesPerSecond = getAngularVelocityDegreesPerSecond();
    }

    private double getAngularVelocityDegreesPerSecond() {
        double currentAngleDegrees = getTurretAngleRobotRelative();
        double deltaDegrees = currentAngleDegrees - lastTurretAngleDegrees;

        // if the change in angle is greater than 180 degrees, we have wrapped around the encoder, so we need to adjust the delta accordingly
        if (deltaDegrees > 180.0) {
            deltaDegrees -= 360.0;
        } else if (deltaDegrees < -180.0) {
            deltaDegrees += 360.0;
        }

        lastTurretAngleDegrees = currentAngleDegrees;

        // calculate angular velocity in degrees per second
        double angularVelocityDegreesPerSecond = deltaDegrees / 0.02; // assuming this method is called every 20 ms

        return angularVelocityDegreesPerSecond;
    }

    // CONFIGURATIONS

    /**
     * Get the motor configurations for the turret motor.
     * @return the motor configurations for the turret motor
     */
    private Slot0Configs getTurretMotionConfigs() {
        Slot0Configs configs = new Slot0Configs();

        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;

        configs.kS = SLOT_kS;
        configs.kV = SLOT_kV;

        return configs;
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
     * Get the software limit switch configurations for the turret motor.
     * Limits are based on Turret.TURRET_MAX_ANGLE_DEGREES constant.
     * @return the software limit switch configurations for the turret motor
     */
    private SoftwareLimitSwitchConfigs getTurretSoftwareLimitConfigs() {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();

        // Convert max angle degrees to mechanism rotations
        // SensorToMechanismRatio is set so 1 mechanism rotation = 180 degrees
        // Therefore: mechanismRotations = degrees / 180
        double limitRotations = Turret.TURRET_MAX_ANGLE_DEGREES / 180.0;

        configs.ForwardSoftLimitEnable = true;
        configs.ForwardSoftLimitThreshold = limitRotations;

        configs.ReverseSoftLimitEnable = true;
        configs.ReverseSoftLimitThreshold = -limitRotations;

        return configs;
    }

    public void changeTurretTrim(double deltaDegrees) {
        this.turretTrimDegrees += deltaDegrees;
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
        // configs.MagnetSensor.MagnetOffset = Turret.PINION_ENCODER_OFFSET;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return configs;
    }

    public CANcoderConfiguration getFollowerEncoderConfigs() {
        CANcoderConfiguration configs = getEncoderConfigs();
        // configs.MagnetSensor.MagnetOffset = Turret.FOLLOWER_ENCODER_OFFSET;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return configs;
    }

    // CRT METHODS

    /**
     * Calculates the continuous position of the pinion (driving) encoder in revolutions
     * @return the continuous position of the pinion encoder in revolutions
     */
    @AutoLogOutput(key = "Turret/PinionEncoderRevsCalculated")
    private double getPinionEncoderRevs() {
        double pinionEncoderReading = positiveMod(this.pinionEncoderSignal.getValueAsDouble(), 1.0);
        double followerEncoderReading = positiveMod(this.followerEncoderSignal.getValueAsDouble(), 1.0);

        double bestError = Double.MAX_VALUE;
        double bestPosition = lastPinionRevs;

        for (int k = -15; k <= 15; k++) {
            double assumedPinionRevs = pinionEncoderReading + k;

            double predictedFollowerReading =
                positiveMod(assumedPinionRevs * (Turret.PINION_ENCODER_TEETH / Turret.FOLLOWER_ENCODER_TEETH), 1.0);

            double predictionError = Math.abs(predictedFollowerReading - followerEncoderReading);
            if (predictionError > 0.5) {
                predictionError = 1.0 - predictionError;
            }

            double continuityError = Math.abs(assumedPinionRevs - lastPinionRevs);
            double score = predictionError + continuityError * 0.001;   

            if (score < bestError) {
                bestError = score;
                bestPosition = assumedPinionRevs;
            }
        }

        lastPinionRevs = bestPosition;
        return bestPosition;
    }

    /**
     * Calculates the continuous position of  the turret in revolutions of the entire mechanism.
     * @return the continuous position of the turret in revolutions.
     */
    private double getTurretPositionRevs() {
        double rawPinionRevs = getPinionEncoderRevs();
        double rawTurretRevs = rawPinionRevs * (Turret.PINION_ENCODER_TEETH / Turret.TURRET_GEAR_TEETH); // convert pinion revolutions to turret revolutions

        double wrapped = positiveMod(rawTurretRevs, Turret.ENCODER_COMBINED_PERIOD_TURRET_REV);

        if (!isInitialized) {
            this.lastPositionRevs = wrapped;
            isInitialized = true;
            return wrapped;
        }

        double delta = wrapped - this.lastPositionRevs;

        // if the change in position is greater than half the combined period, we have wrapped around the encoder, so we need to adjust the delta accordingly
        if (delta > Turret.ENCODER_COMBINED_PERIOD_TURRET_REV  / 2.0) {
            delta -= Turret.ENCODER_COMBINED_PERIOD_TURRET_REV ;
        } else if (delta < -Turret.ENCODER_COMBINED_PERIOD_TURRET_REV  / 2.0) {
            delta += Turret.ENCODER_COMBINED_PERIOD_TURRET_REV ;
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
     * Gets the angle of the turret in turret space wrapped to the turret range.
     * @return the angle of the turret in turret space, wrapped to [-TURRET_MAX_ANGLE_DEGREES, +TURRET_MAX_ANGLE_DEGREES)
     */
    private double getTurretAngle() {
        double continuousRevs = getTurretPositionRevs();
        double turretAngleDegrees = revsToDegreesContinuous(continuousRevs);
        
        double turretAngleWithOffset = turretAngleDegrees - Turret.TURRET_CENTER_OFFSET_DEG;

        // apply offset and wrap to turret range
        return wrap(turretAngleWithOffset, -Turret.TURRET_MAX_ANGLE_DEGREES, Turret.TURRET_MAX_ANGLE_DEGREES);
    }

    /**
     * Gets the angle of the turret in robot space.
     * @return the angle of the turret in robot space, wrapped to turret range
     */
    public double getTurretAngleRobotRelative() {
        return wrap(getTurretAngle() - Turret.TURRET_ROBOT_OFFSET_DEG, 
            -Turret.TURRET_MAX_ANGLE_DEGREES, Turret.TURRET_MAX_ANGLE_DEGREES);
    }

    /**
     * Gets the angle of the turret in field space.
     * @return the angle of the turret in field space, wrapped to [-180, 180) for field coordinates
     */
    private double getTurretAngleFieldRelative() {
        double robotRelativeAngle = getTurretAngleRobotRelative();
        double currentRobotHeading = this.drive.getPose().getRotation().getDegrees();

        double fieldCentricContinuous = robotRelativeAngle + currentRobotHeading;

        // Field-relative angles should still wrap at 180 since field is 360 degrees
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
     * Wraps the input angle to be within the turret's configured range 
     * [-TURRET_MAX_ANGLE_DEGREES, +TURRET_MAX_ANGLE_DEGREES).
     * Includes hysteresis to prevent oscillation at the wrap boundaries.
     * @param input the angle to wrap
     * @param currentTurretAngle the current actual turret angle (for hysteresis)
     * @return the wrapped angle within the turret range
     */
    private double wrapToTurretRange(double input, double currentTurretAngle) {
        double maxAngle = Turret.TURRET_MAX_ANGLE_DEGREES;
        
        // First, normalize input to [-180, +180) to get canonical direction
        double normalizedInput = wrap(input, -180, 180);
        
        // Calculate both possible target angles (could go positive or negative way)
        double positiveOption = normalizedInput;
        double negativeOption = normalizedInput;
        
        // Adjust to get the two possible representations within turret range
        if (normalizedInput < 0) {
            positiveOption = normalizedInput + 360; // e.g., -170 -> +190
        } else {
            negativeOption = normalizedInput - 360; // e.g., +170 -> -190
        }
        
        // Check which options are within the turret's physical limits
        boolean positiveValid = positiveOption >= -maxAngle && positiveOption <= maxAngle;
        boolean negativeValid = negativeOption >= -maxAngle && negativeOption <= maxAngle;
        
        double wrapped;
        
        if (positiveValid && negativeValid) {
            // Both options are valid - choose based on current position and hysteresis
            double distToPositive = Math.abs(currentTurretAngle - positiveOption);
            double distToNegative = Math.abs(currentTurretAngle - negativeOption);
            
            // Use hysteresis: prefer staying on current side unless the other side is 
            // significantly closer (by more than hysteresis margin)
            if (currentTurretAngle >= 0) {
                // Currently on positive side - prefer positive unless negative is much closer
                if (distToNegative + WRAP_HYSTERESIS_DEGREES < distToPositive) {
                    wrapped = negativeOption;
                } else {
                    wrapped = positiveOption;
                }
            } else {
                // Currently on negative side - prefer negative unless positive is much closer
                if (distToPositive + WRAP_HYSTERESIS_DEGREES < distToNegative) {
                    wrapped = positiveOption;
                } else {
                    wrapped = negativeOption;
                }
            }
        } else if (positiveValid) {
            wrapped = positiveOption;
        } else if (negativeValid) {
            wrapped = negativeOption;
        } else {
            // Neither option is in range - clamp to nearest limit
            // This shouldn't happen with a ±200° range, but handle it safely
            wrapped = Math.max(-maxAngle, Math.min(maxAngle, normalizedInput));
        }
        
        Logger.recordOutput("Turret/CurrentAngle", currentTurretAngle);
        Logger.recordOutput("Turret/WrappedAngle", wrapped);
        
        return wrapped;
    }
    
    /**
     * Simple wrap to [-180, 180) without hysteresis.
     * Used for display/logging purposes only.
     * @param input the angle to wrap
     * @return the wrapped angle within [-180, 180)
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
