package frc.robot.subsystems.trigger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;
import frc.robot.MotorCurrentLimits;

public class TriggerIOTalonFX implements IORefresher, TriggerIO {
    private final TalonFX motor;             // Motor object
    private final BaseStatusSignal motorRps; // Rotations per second
    private final DigitalInput proximitySensor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);

    public TriggerIOTalonFX(int proxChannel) {
        this.motor = new TalonFX(CanID.TRIGGER_MOTOR.getID());
        this.motorRps = motor.getRotorVelocity();
        this.proximitySensor = new DigitalInput(proxChannel);

        // set motor pids
        Slot0Configs configs = new Slot0Configs();

        configs.kP = 0.1;
        configs.kD = 0.001;
        configs.kS = 0.0;
        configs.kV = 0.1;

        motor.getConfigurator().apply(configs);
        motor.getConfigurator().apply(MotorCurrentLimits.TRIGGER.toCurrentLimitsConfigs());

        motorRps.setUpdateFrequency(50);

        motor.optimizeBusUtilization();
    }

    // Refresh all signals
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorRps);
    }

    @Override
    public void updateInputs(TriggerIOInputs inputs) {
        inputs.motorRps = motorRps.getValueAsDouble();
        inputs.proximitySensor = proximitySensor.get();
    }

    @Override
    public void setSpeed(double rps) {
        motor.setControl(velocityControl.withVelocity(rps));
    }
}