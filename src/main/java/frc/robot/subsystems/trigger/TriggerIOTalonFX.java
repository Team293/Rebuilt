package frc.robot.subsystems.trigger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class TriggerIOTalonFX implements IORefresher, TriggerIO {
    private final TalonFX motor;             // Motor object
    private final BaseStatusSignal motorRps; // Rotations per second
    private final DigitalInput proximitySensor;

    public TriggerIOTalonFX(int proxChannel) {
        this.motor = new TalonFX(CanID.TRIGGER_MOTOR.getID());
        this.motorRps = motor.getRotorVelocity();
        this.proximitySensor = new DigitalInput(proxChannel);
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
        motor.set(rps);
    }
}