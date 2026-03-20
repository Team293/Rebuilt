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

public class ShooterIOTalonFX implements ShooterIO {
    private static final double HOOD_ZERO_CURRENT = 1.75; // amps at which we consider the hood to have hit a limit
    private static final double FLYWHEEL_SETPOINT_UPDATE_DEADBAND_RPS = 0.35; // ignore tiny target changes

    private final TalonFX flywheelMotor;
    private final TalonFXS hoodMotor;
    private final CANcoder hoodEncoder; // using wcp throughbore; you interface through 'CANcoder' class

    private final VelocityTorqueCurrentFOC flywheelControl = new VelocityTorqueCurrentFOC(0);

    private final VoltageOut hoodZeroingControl = new VoltageOut(-4);

    private final PositionVoltage hoodPositionControl = new PositionVoltage(0);
    private final StatusSignal<Voltage> hoodMotorVoltage; // volts
    private final StatusSignal<AngularVelocity> motorVelocity; // rps
    private final StatusSignal<Angle> hoodMotorPosition; // motor rotations
    private final StatusSignal<Angle> hoodAngle; // degrees
    private final StatusSignal<Current> hoodMotorCurrent; // amps

    private double hoodAngleSetPoint = 0.0;
    private double flywheelRPSSetPoint = 0.0;
    private double lastAppliedFlywheelRPSSetPoint = Double.NaN;

    private double hoodTargetEncoder = 0.0;
    private boolean isZeroing = true;

    private double distanceTrim = 0.0; // minor adjustment to the returned distance based on operator controller input, in degrees

    public ShooterIOTalonFX() {
        this.hoodMotor = new TalonFXS(CanID.HOOD_MOTOR.getID());
        this.hoodEncoder = new CANcoder(CanID.HOOD_ENCODER.getID());
        this.flywheelMotor = new TalonFX(CanID.FLYWHEEL_MOTOR.getID());

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
        flywheelSlot0.kP = 20; // amps / rps of error
        flywheelSlot0.kI = 0.0;
        flywheelSlot0.kD = 0.0;
        flywheelSlot0.kS = 0.0; // amps needed to overcome static friction
        flywheelSlot0.kV = 0.0; // not used for torque control

        this.flywheelMotor.getConfigurator().apply(flywheelSlot0);

        this.motorVelocity = flywheelMotor.getVelocity();
        this.hoodAngle = hoodEncoder.getAbsolutePosition();
        this.hoodMotorPosition = hoodMotor.getPosition();
        this.hoodMotorVoltage = hoodMotor.getMotorVoltage();
        this.hoodMotorCurrent = hoodMotor.getSupplyCurrent();

        // force refresh before zero calculations
        BaseStatusSignal.refreshAll(motorVelocity, hoodAngle, hoodMotorPosition, hoodMotorVoltage, hoodMotorCurrent);

        flywheelMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
    }

    @Override
    public void runZeroingHood() {
        hoodMotor.setControl(hoodZeroingControl); // move hood down at a slow speed

        if (hoodMotorCurrent.getValueAsDouble() > HOOD_ZERO_CURRENT) { // if we hit the floor, the current will spike up
            hoodMotor.stopMotor();
            hoodEncoder.setPosition(0); // set encoder position to 0 when we hit the limit
            isZeroing = false;
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

    /**
     * Periodically refreshes encoder signals.
     * Called automatically by the subsystem data refresher.
     */
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorVelocity, hoodAngle, hoodMotorPosition, hoodMotorCurrent, hoodMotorVoltage);
    }

    /**
     * Periodically called to update the shooter information for logging
     *
     * @param inputs ShooterIOInputs object to update
     */
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRPS = this.motorVelocity.getValueAsDouble();
        inputs.hoodAngleDeg = this.hoodAngle.getValueAsDouble() * 30.0 + 15.0; // convert [0,1] encoder rotations to degrees [15, 45]
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
        this.flywheelRPSSetPoint = rps;

        // boolean firstCommand = Double.isNaN(lastAppliedFlywheelRPSSetPoint);
        // boolean meaningfulChange = firstCommand
        //         || (Math.abs(rps - lastAppliedFlywheelRPSSetPoint) >= FLYWHEEL_SETPOINT_UPDATE_DEADBAND_RPS);

        // if (!meaningfulChange) {
        //     return;
        // }

        this.flywheelControl.withVelocity(rps);
        flywheelMotor.setControl(this.flywheelControl);
        this.lastAppliedFlywheelRPSSetPoint = rps;
    }
    /**
     * Set the target hood angle.
     * @param angle target angle
     */
    public void setHoodAngle(double angle) {
        angle = Math.max(15.0, Math.min(45.0, angle));
        this.hoodAngleSetPoint = angle;
        angle = Math.max(15, Math.min(45, angle));
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
    private double angleToEncoder(double angle) {
        angle -= 14.0; // shift so that 0 is at 15 degrees
        angle /= 30.0; // scale so that 1 is at 45 degrees
        if (angle < 0.0) {
            return 0.0;
        } else return Math.min(angle, 1.0);
    }
}