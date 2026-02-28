package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.lib.subsystem.IORefresher;

public class ClimberIOTalonFX implements ClimberIO, IORefresher {
    private final int climberMotorID = 10; // CAN ID for the climber motor
    private ClimberIOInputs climberIOInputs;
    private TalonFX climberMotor;

    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<Current> climberCurrent;
    private final StatusSignal<AngularVelocity> climberVelocity;

    private static VelocityVoltage velocityControl = new VelocityVoltage(0.0d).withSlot(0);
    private static PositionVoltage positionControl = new PositionVoltage(0.0d).withSlot(0);

    public ClimberIOTalonFX() {
        climberMotor = new TalonFX(climberMotorID);
        climberIOInputs = new ClimberIOInputs();

        climberMotor.getConfigurator().apply(getClimberMotorConfig());

        climberPosition = climberMotor.getPosition();
        climberCurrent = climberMotor.getStatorCurrent();
        climberVelocity = climberMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, climberPosition, climberCurrent, climberVelocity);

        climberMotor.optimizeBusUtilization();
    }

    // Fetches data from the motors
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(climberPosition, climberCurrent);
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPositionRotations = climberPosition.getValueAsDouble(); // Get position in rotations
        inputs.climberCurrentAmps = climberCurrent.getValueAsDouble(); // Get current in amps
    }

    public double getVelocity() {
        return climberVelocity.getValueAsDouble(); // Get velocity in rotations per second
    }

    public void setVelocity(double velocityRotationsPerSecond) {
        // Create a controller for the motor
        velocityControl.withVelocity(velocityRotationsPerSecond).withSlot(0);
        climberMotor.setControl(velocityControl);
    }

    public double getPosition() {
        return climberPosition.getValueAsDouble(); // Get position in rotations
    }

    public void setPosition(double positionRotations) {
        // Create a controller for the motor
        positionControl.withPosition(positionRotations).withSlot(0);
        climberMotor.setControl(positionControl);
    }

    // Method to configure the climber motor settings
    public static TalonFXConfiguration getClimberMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Configure the motor settings here (e.g., PID, current limits, etc.)

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0.1; // Example PID configuration
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0; // Gravity compensation term
        config.Slot0.kS = 0.0; // Static friction compensation term, if needed
        config.Slot0.kV = 0.0; // Voltage term, needs to be tuned based on the system
        
        return config;
    }
}
