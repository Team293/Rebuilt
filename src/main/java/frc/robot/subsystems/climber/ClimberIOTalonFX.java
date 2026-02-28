package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    public ClimberIOTalonFX() {
        climberMotor = new TalonFX(climberMotorID);
        climberIOInputs = new ClimberIOInputs();

        climberMotor.getConfigurator().apply(getClimberMotorConfig());

        climberPosition = climberMotor.getPosition();
        climberCurrent = climberMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, climberPosition, climberCurrent);

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
