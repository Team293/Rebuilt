package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class IntakeIOTalonFX implements IntakeIO, IORefresher {
    private static final double DEPLOY_SPEED = 1.0; //placeholder values 
    private static final double RETRACT_SPEED = -1.0;

    private final TalonFX intakeMotor;
    private final TalonFX deployMotor;

    private final StatusSignal<AngularVelocity> intakeVelocity; // Status Signal Fixed
    private final StatusSignal<Current> intakeCurrent;
    private final StatusSignal<AngularVelocity> deployVelocity;
    private final StatusSignal<Current> deployCurrent;

    private IntakeIOInputs latestInputs;

    public IntakeIOTalonFX() { 
        intakeMotor = new TalonFX(CanID.INTAKE_MOTOR.getID()); // get intake canID
        deployMotor = new TalonFX(CanID.INTAKE_DEPLOY_MOTOR.getID()); // get deploy canID

         // Configure motors

        var deployConfig = new TalonFXConfiguration(); // Deploy Motor to break mode when no power
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotor.getConfigurator().apply(deployConfig);

        intakeVelocity = intakeMotor.getVelocity();
        intakeCurrent = intakeMotor.getStatorCurrent();
        deployVelocity = deployMotor.getVelocity();
        deployCurrent = deployMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, // set to 50hz to match robot loop and avoid stale data issues
                intakeVelocity, intakeCurrent, deployVelocity, deployCurrent);

        intakeMotor.optimizeBusUtilization();
        deployMotor.optimizeBusUtilization();
    }

    // Fetches data from the motors
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeCurrent, deployVelocity, deployCurrent);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        latestInputs = inputs;
        inputs.intakeVelocityRPS = intakeVelocity.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.deployVelocityRPS = deployVelocity.getValueAsDouble();
        inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();
    }

    @Override
    public void on(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void off() {
        intakeMotor.set(0.0);
    }

    // Extends the intake out
    @Override
    public void deploy() {
        if (latestInputs != null) {
            latestInputs.deployed = true; //mark as deployed 
        }
        deployMotor.set(DEPLOY_SPEED);
    }

    // Pulls the intake back in
    @Override
    public void retract() {
        if (latestInputs != null) {
            latestInputs.deployed = false; //mark as retracted  w
        }
        deployMotor.set(RETRACT_SPEED);
    }
}
