package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class ShooterIOTalonFX implements IORefresher, ShooterIO {
    private static final double HOOD_GEAR_RATIO = 1.0/1.0; // hood pulley teeth / motor pulley teeth (motor revs per hood rev)

    private final TalonFX flywheelMotor;
    private final TalonFXS hoodMotor;
    private final CANcoder hoodEncoder; // using wcp throughbore; you interface through 'CANcoder' class

    private final double hoodMotorPositionOffset; // offset in motor rotations, calculated at startup, units in motor rotations

    private final VelocityVoltage flywheelVelocityControl = new VelocityVoltage(0);
    private final PositionVoltage hoodPositionControl = new PositionVoltage(0);

    private final StatusSignal<AngularVelocity> motorVelocity; // rps
    private final StatusSignal<Angle> hoodMotorPosition; // motor rotations
    private final StatusSignal<Angle> hoodAngle; // degrees

    private double hoodAngleSetPoint = 0.0;
    private double flywheelRPSSetPoint = 0.0;

    public ShooterIOTalonFX() {
        // hood configs
        this.hoodMotor = new TalonFXS(CanID.HOOD_MOTOR.getID());

        var hoodSlot0 = new Slot0Configs();
        hoodSlot0.kP = 0.1;
        hoodSlot0.kI = 0.0;
        hoodSlot0.kD = 0.0;
        hoodSlot0.kS = 0.0;
        hoodSlot0.kV = 0.0;

        this.hoodMotor.getConfigurator().apply(hoodSlot0);

        // flywheel configs
        this.flywheelMotor = new TalonFX(CanID.FLYWHEEL_MOTOR.getID());

        var flywheelSlot0 = new Slot0Configs();
        flywheelSlot0.kP = 0.1;
        flywheelSlot0.kI = 0.0;
        flywheelSlot0.kD = 0.0;
        flywheelSlot0.kS = 0.0;
        flywheelSlot0.kV = 0.0;

        this.flywheelMotor.getConfigurator().apply(flywheelSlot0);

        this.hoodEncoder = new CANcoder(CanID.HOOD_ENCODER.getID());

        CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
        // constrains the range to [0, 1)
        hoodEncoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);

        this.hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

        this.motorVelocity = flywheelMotor.getVelocity();
        this.hoodAngle = hoodEncoder.getAbsolutePosition();
        this.hoodMotorPosition = hoodMotor.getPosition();

        // force refresh before zero calculations
        BaseStatusSignal.refreshAll(motorVelocity, hoodAngle, hoodMotorPosition);

        // convert hood angle to motor rotations and calculate offset
        this.hoodMotorPositionOffset = this.hoodMotorPosition.getValueAsDouble() - angleToMotorRotations(this.hoodAngle.getValueAsDouble());

        flywheelMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
    }
    
    /**
     * Periodically refreshes encoder signal
     * @note This is called automatically
     */
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorVelocity, hoodAngle, hoodMotorPosition);
    }

    /**
     * Periodically called to update the shooter information for logging
     * @param inputs ShooterIOInputs object to update
     */
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorRPS = this.motorVelocity.getValueAsDouble();
        inputs.hoodAngle = this.hoodAngle.getValueAsDouble() * 360; // convert rotations to degrees
        inputs.hoodMotorPosition = this.hoodMotorPosition.getValueAsDouble();

        inputs.flywheelSetPointRPS = this.flywheelRPSSetPoint;
        inputs.hoodSetPointAngle = this.hoodAngleSetPoint;
    }

    /**
     * Set the target flywheel velocity
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
     * @param angle target angle TODO: relative to what 
     */
    @Override
    public void setHoodAngle(double angle) {
        this.hoodAngleSetPoint = angle;
        double motorRotations = angleToMotorRotations(angle);
        this.hoodPositionControl.withPosition(hoodMotorPositionOffset + motorRotations);

        this.hoodMotor.setControl(this.hoodPositionControl);
    }

    /**
     * Converts angle in degrees to motor rotations per second
     * @param angle Input angle in degrees
     * @return double motor rotations per second
     */
    private static double angleToMotorRotations(double angle) {
        // rotations = (angle_deg * gear_ratio) / 360
        return angle * HOOD_GEAR_RATIO / 360.0;
    }
}
