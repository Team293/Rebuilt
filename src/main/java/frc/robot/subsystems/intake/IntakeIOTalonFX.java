package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeIOTalonFX implements IntakeIO, IORefresher {
    // TalonFX Motors
    private final TalonFX intakeMotor;
    // private final TalonFX deployMotor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0.0); 

    // Status Signals
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Current> intakeCurrent;
    // private final StatusSignal<AngularVelocity> deployVelocity;
    // private final StatusSignal<Current> deployCurrent;

    // Inputs for logging
    private IntakeIOInputs intakeIO;

    // IntakeIOTalonFX constructor
    public IntakeIOTalonFX() {
        intakeMotor = new TalonFX(CanID.INTAKE_MOTOR.getID(), "Canivore_Drivetrain"); // Setup the intake motor with the CAN ID
        // deployMotor = new TalonFX(CanID.INTAKE_DEPLOY_MOTOR.getID()); // Set up deploy motor with the CAN ID

        // Configure motors
        // deployMotor.getConfigurator().apply(getDeployMotorConfig());
        intakeMotor.getConfigurator().apply(getIntakeMotorConfig());

        // Configure motor signals
        intakeVelocity = intakeMotor.getVelocity();
        intakeCurrent = intakeMotor.getStatorCurrent();
        // deployVelocity = deployMotor.getVelocity();
        // deployCurrent = deployMotor.getStatorCurrent();

        intakeMotor.optimizeBusUtilization();
        // deployMotor.optimizeBusUtilization();
    }

    // Fetches data from the motors
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeCurrent);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRPS = intakeVelocity.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        // inputs.deployVelocityRPS = deployVelocity.getValueAsDouble();
        // inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();
        intakeIO = inputs;
    }

    // Set the speed of the intake motor
    // double speed - Speed to set the motor to in Rotations Per Second
    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    // Set the speed of the deploy motor
    // double speed - Speed to set the motor to in Rotations Per Second
    @Override
    public void setDeploySpeed(double speed) {
        this.velocityControl.withVelocity(speed);
        // deployMotor.setControl(this.velocityControl);
    }

    // Return the state of the Intake
    @Override
    public IntakeState getIntakeState() {
        return intakeIO.intakeState;
    }

    // Set the state of the Intake
    @Override
    public void setIntakeState(IntakeState newState) {
        intakeIO.intakeState = newState;
    }

    // Return the current of the deploy motor in Amps
    @Override
    public double getDeployMotorCurrent() {
        return intakeIO.deployCurrentAmps;
    }

    public static TalonFXConfiguration getIntakeMotorConfig() {
        var intakeConfig = new TalonFXConfiguration();

        // Set Intake motor to Coast when not on
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Intake motor PID values
        intakeConfig.Slot0.kP = 1;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;

        return intakeConfig;
    }

    public static TalonFXConfiguration getDeployMotorConfig() {
        var deployConfig = new TalonFXConfiguration();

        // Set deploy motor to brake mode when not on
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Deploy motor PID values
        deployConfig.Slot0.kP = 0.1;
        deployConfig.Slot0.kI = 0.0;
        deployConfig.Slot0.kD = 0.0;

        return deployConfig;
    }
}
