package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class IntakeIOTalonFX implements IORefresher, IntakeIO {
    private final TalonFX motor;
    private final BaseStatusSignal motorRps;

    public IntakeIOTalonFX() {
        this.motor = new TalonFX(CanID.INTAKE.getID());
        this.motorRps = motor.getRotorVelocity();
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorRps);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorRps = motorRps.getValueAsDouble();
    }

    @Override
    public void setSpeed(double rps) {
        motor.set(rps);
    }
}
