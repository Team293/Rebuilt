package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.CanID;

public class IntakeIOTalonFX implements IntakeIO {
    private static final double DEPLOY_SPEED = 1.0; //these values are placeholders, will need to be tuned once we have the prototype built
    private static final double RETRACT_SPEED = -1.0;

    private final TalonFX intakeMotor;
    private final TalonFX deployMotor;

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    private final StatusSignal<Double> intakeVelocity;
    private final StatusSignal<Double> intakeCurrent;
    private final StatusSignal<Double> deployVelocity;
    private final StatusSignal<Double> deployCurrent;

    private boolean deployed = false;

    public IntakeIOTalonFX() {
        intakeMotor = new TalonFX(CanID.INTAKE_MOTOR.getID()); // get can ID for motors
        deployMotor = new TalonFX(CanID.INTAKE_DEPLOY_MOTOR.getID());

        var deployConfig = new TalonFXConfiguration();
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotor.getConfigurator().apply(deployConfig);

        intakeVelocity = intakeMotor.getVelocity();
        intakeCurrent = intakeMotor.getStatorCurrent();
        deployVelocity = deployMotor.getVelocity();
        deployCurrent = deployMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
                intakeVelocity, intakeCurrent, deployVelocity, deployCurrent);

        intakeMotor.optimizeBusUtilization();
        deployMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeCurrent, deployVelocity, deployCurrent);

        inputs.intakeVelocityRPS = intakeVelocity.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.deployVelocityRPS = deployVelocity.getValueAsDouble();
        inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();
        inputs.deployed = deployed;
    }

    @Override
    public void on(double speed) {
        intakeMotor.setControl(dutyCycle.withOutput(speed));
    }

    @Override
    public void off() {
        intakeMotor.setControl(dutyCycle.withOutput(0.0));
    }

    @Override
    public void deploy() {
        deployed = true;
        deployMotor.setControl(dutyCycle.withOutput(DEPLOY_SPEED));
    }

    @Override
    public void retract() {
        deployed = false;
        deployMotor.setControl(dutyCycle.withOutput(RETRACT_SPEED));
    }
}
