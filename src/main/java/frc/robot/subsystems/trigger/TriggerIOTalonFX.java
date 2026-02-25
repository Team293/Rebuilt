package frc.robot.subsystems.trigger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class TriggerIOTalonFX implements IORefresher, TriggerIO {
    private final TalonFX motor;             // Motor object
    private final BaseStatusSignal motorRps; // Rotations per second

    public TriggerIOTalonFX() {
        this.motor = new TalonFX(CanID.TRIGGER_MOTOR.getID());
        this.motorRps = motor.getRotorVelocity();
    }

    // Refresh all signals
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorRps);
    }

    @Override
    public void updateInputs(TriggerIOInputs inputs) {
        inputs.motorRps = motorRps.getValueAsDouble();
    }

    @Override
    public void setSpeed(double rps) {
        motor.set(rps);
    }
}