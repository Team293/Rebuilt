package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CanID;
import frc.robot.MotorCurrentLimits;

public class ShooterIOTalonFX implements ShooterIO {
    private static final double HOOD_ZERO_CURRENT = 1.75; // amps at which we consider the hood to have hit a limit
    private static final double HOOD_MIN_ANGLE_DEG = 15.0;
    private static final double HOOD_MAX_ANGLE_DEG = 45.0;
    private static final double HOOD_DEGREES_PER_ROTATION = HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG;
    private static final double FLYWHEEL_HOLD_KP = 14.0; // amps / rps
    private static final double FLYWHEEL_RECOVERY_KP = FLYWHEEL_HOLD_KP;
    private static final double FLYWHEEL_RISING_TARGET_KP = 16.0; // amps / rps; small increase over holding
    private static final double RISING_TARGET_MIN_STEP_RPS = 0.02;
    private static final double RISING_TARGET_MIN_ERROR_RPS = 3.0;
    private static final double FLYWHEEL_KS_AMPS = 15.0;
    private static final double RECOVERY_ENTER_ERROR_RPS = 3.0;
    private static final double RECOVERY_EXIT_ERROR_RPS = 1.5;

    private final TalonFX flywheelMotor;
    private final TalonFXS hoodMotor;
    private final CANcoder hoodEncoder; // using wcp throughbore; you interface through 'CANcoder' class

    private final VelocityTorqueCurrentFOC flywheelControl = new VelocityTorqueCurrentFOC(0);

    private final VoltageOut hoodZeroingControl = new VoltageOut(-4);

    private final PositionVoltage hoodPositionControl = new PositionVoltage(0);
    private final StatusSignal<Voltage> hoodMotorVoltage; // volts
    private final StatusSignal<AngularVelocity> motorVelocity; // rps
    private final StatusSignal<Current> flywheelStatorCurrent;
    private final StatusSignal<Current> flywheelSupplyCurrent;
    private final StatusSignal<Voltage> flywheelVoltage;
    private final StatusSignal<Angle> hoodEncoderPosition; // homed CANcoder rotations, also used for remote feedback
    private final StatusSignal<Current> hoodMotorCurrent; // amps

    private double hoodAngleSetPoint = 0.0;
    private double flywheelRPSSetPoint = 0.0;
    private boolean flywheelRecovering = false;

    private double hoodTargetEncoder = 0.0;
    private boolean isZeroing = true;

    private double distanceTrim = 0.3; // minor adjustment to the returned distance based on operator controller input, in degrees

    public ShooterIOTalonFX() {
        this.hoodMotor = new TalonFXS(CanID.HOOD_MOTOR.getID());
        this.hoodEncoder = new CANcoder(CanID.HOOD_ENCODER.getID());
        this.flywheelMotor = new TalonFX(CanID.FLYWHEEL_MOTOR.getID());

        this.hoodMotor.getConfigurator().apply(MotorCurrentLimits.hood());
        this.flywheelMotor.getConfigurator().apply(MotorCurrentLimits.flywheel());

        // hood encoder configs
        CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
        hoodEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        this.hoodEncoder.getConfigurator().apply(hoodEncoderConfig); // apply default configs to encoder before
                                                                            // using it for feedback
        // constrains the range to [0, 1)
        hoodEncoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        this.hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

        // hood configs
        var hoodSlot0 = new Slot0Configs();
        hoodSlot0.kP = 100.0;
        hoodSlot0.kI = 0.0;
        hoodSlot0.kD = 0.0;
        hoodSlot0.kS = 20.0;
        hoodSlot0.kV = 0.0;

        SoftwareLimitSwitchConfigs hoodSoftLimits = new SoftwareLimitSwitchConfigs();
        hoodSoftLimits.ForwardSoftLimitEnable = false;
        hoodSoftLimits.ForwardSoftLimitThreshold = 1.1;
        hoodSoftLimits.ReverseSoftLimitEnable = false;
        hoodSoftLimits.ReverseSoftLimitThreshold = -0.1;

        MotorOutputConfigs hoodMotorOutputConfigs = new MotorOutputConfigs();
        hoodMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        ExternalFeedbackConfigs hoodExternalFeedback = new ExternalFeedbackConfigs();
        hoodExternalFeedback.withRemoteCANcoder(this.hoodEncoder);

        CommutationConfigs hoodCommutation = new CommutationConfigs();
        hoodCommutation.MotorArrangement = MotorArrangementValue.Brushed_DC;

        this.hoodMotor.getConfigurator().apply(hoodSlot0);
        this.hoodMotor.getConfigurator().apply(hoodExternalFeedback);
        this.hoodMotor.getConfigurator().apply(hoodSoftLimits);
        this.hoodMotor.getConfigurator().apply(hoodCommutation);
        this.hoodMotor.getConfigurator().apply(hoodMotorOutputConfigs);

        // flywheel configs (units in AMPS)
        var flywheelSlot0 = new Slot0Configs();
        flywheelSlot0.kP = FLYWHEEL_HOLD_KP;
        flywheelSlot0.kI = 0.0;
        flywheelSlot0.kD = 0.0;
        flywheelSlot0.kS = FLYWHEEL_KS_AMPS;
        flywheelSlot0.kV = 0.0; // not used for torque control

        this.flywheelMotor.getConfigurator().apply(flywheelSlot0);

        // Recovery uses the original holding gain. Extra gain is confined to
        // slot 2 while the target is actively rising and speed is well below it.
        var flywheelSlot1 = new Slot1Configs();
        flywheelSlot1.kP = FLYWHEEL_RECOVERY_KP;
        flywheelSlot1.kI = 0.0;
        flywheelSlot1.kD = 0.0;
        flywheelSlot1.kS = FLYWHEEL_KS_AMPS;
        flywheelSlot1.kV = 0.0;
        this.flywheelMotor.getConfigurator().apply(flywheelSlot1);

        // A modest 14 -> 16 increase while moving away raises the RPM target.
        // Stationary holding and ordinary shot recovery use the original gain.
        var flywheelSlot2 = new Slot2Configs();
        flywheelSlot2.kP = FLYWHEEL_RISING_TARGET_KP;
        flywheelSlot2.kI = 0.0;
        flywheelSlot2.kD = 0.0;
        flywheelSlot2.kS = FLYWHEEL_KS_AMPS;
        flywheelSlot2.kV = 0.0;
        this.flywheelMotor.getConfigurator().apply(flywheelSlot2);

        this.motorVelocity = flywheelMotor.getVelocity();
        this.flywheelStatorCurrent = flywheelMotor.getStatorCurrent();
        this.flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();
        this.flywheelVoltage = flywheelMotor.getMotorVoltage();
        motorVelocity.setUpdateFrequency(100.0);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                flywheelStatorCurrent, flywheelSupplyCurrent, flywheelVoltage);
        this.hoodEncoderPosition = hoodEncoder.getPosition();
        this.hoodMotorVoltage = hoodMotor.getMotorVoltage();
        this.hoodMotorCurrent = hoodMotor.getSupplyCurrent();

        // force refresh before zero calculations
        BaseStatusSignal.refreshAll(motorVelocity, flywheelStatorCurrent, flywheelSupplyCurrent,
                flywheelVoltage, hoodEncoderPosition, hoodMotorVoltage, hoodMotorCurrent);

        flywheelMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
    }

    @Override
    public void runZeroingHood() {
        hoodMotor.setControl(hoodZeroingControl); // move hood down at a slow speed

        if (hoodMotorCurrent.getValueAsDouble() > HOOD_ZERO_CURRENT) { // if we hit the floor, the current will spike up
            hoodMotor.stopMotor();
            // Zero the same position used by telemetry and the motor's remote feedback.
            isZeroing = !hoodEncoder.setPosition(0).isOK();
        }
    }

    @Override
    public void zeroHood() {
        isZeroing = true;
    }

    /**
     * Changes the distance trim by a certain amount of degrees. This is used to make minor adjustments to the distance based on operator controller input.
     * @param deltaDistance the amount of meters to change the distance trim by. Positive values add to the distance, and negative values subtract from the distance.
     */
    @Override
    public void changeDistanceTrim(double deltaDistance) {
        this.distanceTrim += deltaDistance;
    }

    @Override
    public void setDistanceTrim(double deltaDistance) {
        this.distanceTrim = deltaDistance;
    }

    @Override
    public double getDistanceTrim() {
        return this.distanceTrim;
    }

    /**
     * Periodically refreshes encoder signals.
     * Called automatically by the subsystem data refresher.
     */
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorVelocity, flywheelStatorCurrent, flywheelSupplyCurrent,
                flywheelVoltage, hoodEncoderPosition, hoodMotorCurrent, hoodMotorVoltage);
    }

    /**
     * Periodically called to update the shooter information for logging
     *
     * @param inputs ShooterIOInputs object to update
     */
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRPS = this.motorVelocity.getValueAsDouble();
        inputs.flywheelStatorCurrentAmps = this.flywheelStatorCurrent.getValueAsDouble();
        inputs.flywheelSupplyCurrentAmps = this.flywheelSupplyCurrent.getValueAsDouble();
        inputs.flywheelMotorVoltage = this.flywheelVoltage.getValueAsDouble();
        inputs.flywheelRecovering = this.flywheelRecovering;
        inputs.flywheelControlSlot = this.flywheelControl.Slot;
        inputs.hoodAngleDeg = encoderToAngle(this.hoodEncoderPosition.getValueAsDouble());
        inputs.hoodMotorCurrentAmps = this.hoodMotorCurrent.getValueAsDouble();

        inputs.flywheelSetPointRPS = this.flywheelRPSSetPoint;
        inputs.hoodAngleSetPointDeg = this.hoodAngleSetPoint;
        inputs.isZeroing = this.isZeroing;
        inputs.distanceTrimMeters = this.distanceTrim;
    }

    /**
     * Set the target flywheel velocity
     *
     * @param rps - target rotations per second
     */
    @Override
    public void setFlywheelVelocity(double rps) {
        double measuredRps = motorVelocity.getValueAsDouble();
        int slot = selectFlywheelSlot(
                rps, flywheelRPSSetPoint, measuredRps, flywheelRecovering);
        this.flywheelRPSSetPoint = rps;
        this.flywheelRecovering = slot != 0;
        this.flywheelControl.withVelocity(rps).withSlot(slot).withFeedForward(0.0);

        if (rps == 0) {
            flywheelMotor.stopMotor();
        } else {
            flywheelMotor.setControl(this.flywheelControl);
        }
    }

    static int selectFlywheelSlot(
            double targetRps, double previousTargetRps, double measuredRps, boolean recovering) {
        // A rising command is the demand caused by increasing shooting distance.
        // Ignore initial spin-up and tiny command noise. Drop the extra gain
        // earlier, leaving normal recovery/holding to finish approaching target.
        if (previousTargetRps > 0.0
                && targetRps - previousTargetRps > RISING_TARGET_MIN_STEP_RPS
                && targetRps - measuredRps > RISING_TARGET_MIN_ERROR_RPS) {
            return 2;
        }
        return shouldRecoverFlywheel(targetRps, measuredRps, recovering) ? 1 : 0;
    }

    static boolean shouldRecoverFlywheel(double targetRps, double measuredRps, boolean recovering) {
        if (targetRps <= 0.0) {
            return false;
        }
        // One-sided error avoids boosting when the target falls or speed is too
        // high. Hysteresis avoids rapidly switching gains around the boundary.
        double errorRps = targetRps - measuredRps;
        return errorRps > (recovering ? RECOVERY_EXIT_ERROR_RPS : RECOVERY_ENTER_ERROR_RPS);
    }
    
    /**
     * Set the target hood angle.
     * @param angle target angle
     */
    @Override
    public void setHoodAngle(double angle) {
        angle = Math.max(HOOD_MIN_ANGLE_DEG, Math.min(HOOD_MAX_ANGLE_DEG, angle));
        this.hoodAngleSetPoint = angle;
        // convert target angle -> encoder rotations
        this.hoodTargetEncoder = angleToEncoder(angle);

        hoodMotor.setControl(hoodPositionControl.withPosition(this.hoodTargetEncoder).withSlot(0));
    }

    /**
     * Returns a position from zero to 1 representing the hood position,
     * where 0 is 15 degrees and 1 is 45 degrees.
     * @param angle angle in degrees, expected to be in the range [15, 45]
     * @return normalized encoder position in the range [0, 1]
     */
    static double angleToEncoder(double angle) {
        return Math.max(0.0, Math.min(1.0,
                (angle - HOOD_MIN_ANGLE_DEG) / HOOD_DEGREES_PER_ROTATION));
    }

    static double encoderToAngle(double rotations) {
        return rotations * HOOD_DEGREES_PER_ROTATION + HOOD_MIN_ANGLE_DEG;
    }
}
