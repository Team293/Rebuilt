package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    private static final double HOOD_GEAR_RATIO = 1.0 / 1.0; // hood pulley teeth / motor pulley teeth (motor revs per
                                                             // hood rev)
    private static final double SOFTWARE_LIMIT_SWITCH_CURRENT_THRESHOLD = 1.75; // amps at which we consider the hood to have hit a limit

    private final TalonFX flywheelMotor;
    private final TalonFXS hoodMotor;
    private final CANcoder hoodEncoder; // using wcp throughbore; you interface through 'CANcoder' class

    private final VelocityVoltage flywheelVelocityControl = new VelocityVoltage(0);
    private final PositionVoltage hoodPositionControl = new PositionVoltage(0);
    private final StatusSignal<Voltage> hoodMotorVoltage; // volts
    private final StatusSignal<AngularVelocity> motorVelocity; // rps
    private final StatusSignal<Angle> hoodMotorPosition; // motor rotations
    private final StatusSignal<Angle> hoodAngle; // degrees
    private final StatusSignal<Current> hoodMotorCurrent; // amps

    private double hoodAngleSetPoint = 0.0;
    private double flywheelRPSSetPoint = 0.0;

    private double hoodTargetEncoder = 0.0;
    private boolean isZeroing = true;

    private double distanceTrim = 0.0; // minor adjustment to the returned distance based on operator controller input, in degrees

    public ShooterIOTalonFX() {
        this.hoodMotor = new TalonFXS(CanID.HOOD_MOTOR.getID());
        this.hoodEncoder = new CANcoder(CanID.HOOD_ENCODER.getID());
        this.flywheelMotor = new TalonFX(CanID.FLYWHEEL_MOTOR.getID());

        // hood encoder configs
        CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
        hoodEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
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

        // flywheel configs
        var flywheelSlot0 = new Slot0Configs();
        flywheelSlot0.kP = 0.15;
        flywheelSlot0.kI = 0.0;
        flywheelSlot0.kD = 0.0;
        flywheelSlot0.kS = 0.225;
        flywheelSlot0.kV = 0.133;

        // current limits changed from
        // 120, 70 to 80, 60

        this.flywheelMotor.getConfigurator().apply(flywheelSlot0);

        this.motorVelocity = flywheelMotor.getVelocity();
        this.hoodAngle = hoodEncoder.getAbsolutePosition();
        this.hoodMotorPosition = hoodMotor.getPosition();
        this.hoodMotorVoltage = hoodMotor.getMotorVoltage();
        this.hoodMotorCurrent = hoodMotor.getSupplyCurrent();

        // force refresh before zero calculations
        BaseStatusSignal.refreshAll(motorVelocity, hoodAngle, hoodMotorPosition, hoodMotorVoltage, hoodMotorCurrent);

        this.hoodEncoder.setPosition(0);

        flywheelMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
    }

    @Override
    public void runZeroingHood() {
        hoodMotor.setControl(new VoltageOut(-4)); // move hood down at a slow speed

        if (hoodMotorCurrent.getValueAsDouble() > SOFTWARE_LIMIT_SWITCH_CURRENT_THRESHOLD) { // if we hit the floor, the current will spike up
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
     * Periodically refreshes encoder signal
     * 
     * @note This is called automatically
     */
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorVelocity, hoodAngle, hoodMotorPosition, hoodMotorCurrent, hoodMotorVoltage);

        // double current = hoodAngle.getValueAsDouble();

        // // if (Math.abs(current - hoodTargetEncoder) < 0.005) {
        // //     hoodMotor.stopMotor();
        // // }
    }

    /**
     * Periodically called to update the shooter information for logging
     * 
     * @param inputs ShooterIOInputs object to update
     */
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorRPS = this.motorVelocity.getValueAsDouble();
        inputs.hoodAngle = this.hoodAngle.getValueAsDouble() * 30.0 + 15.0; // convert rotations to degrees
        inputs.hoodMotorPosition = this.hoodAngle.getValueAsDouble();
        inputs.hoodMotorCurrent = this.hoodMotorCurrent.getValueAsDouble();

        inputs.flywheelSetPointRPS = this.flywheelRPSSetPoint;
        inputs.hoodSetPointAngle = this.hoodAngleSetPoint;
        inputs.isZeroing = this.isZeroing;
        inputs.distanceTrim = this.distanceTrim;
    }

    /**
     * Set the target flywheel velocity
     * 
     * @param rps - target rotations per second
     */
    @Override
    public void setFlywheelVelocity(double rps) {
        this.flywheelRPSSetPoint = rps;
        this.flywheelVelocityControl.withVelocity(rps);

        flywheelMotor.setControl(this.flywheelVelocityControl);
    }

    /**
     * Set the target hood angle
     * 
     * @param angle target angle
     */
    @Override
    public void setHoodAngle(double angle) {
        angle = Math.max(15, Math.min(45, angle));
        this.hoodAngleSetPoint = angle;

        // convert target angle -> encoder rotations
        this.hoodTargetEncoder = angleToEncoder(angle);

        hoodMotor.setControl(hoodPositionControl.withPosition(this.hoodTargetEncoder).withSlot(0));
    }

    /**
     * Converts angle in degrees to motor rotations per second
     * 
     * @param angle Input angle in degrees
     * @return double motor rotations per second
     */
    private static double angleToMotorRotations(double angle) {
        // rotations = (angle_deg * gear_ratio) / 360
        return angle * HOOD_GEAR_RATIO / 360.0;
    }

    /**
     * Returns a position from zero to 1 representing the position of the hood,
     * where 0 is 15 degrees and 1 is 45 degrees
     * 
     * @param angle
     * @return
     */
    private static double angleToEncoder(double angle) {
        angle -= 14.0; // shift so that 0 is at 15 degrees
        angle /= 30.0; // scale so that 1 is at 45 degrees
        if (angle < 0.0) {
            return 0.0;
        } else if (angle > 1.0) {
            return 1.0;
        } else {
            return angle;
        }
    }
}
