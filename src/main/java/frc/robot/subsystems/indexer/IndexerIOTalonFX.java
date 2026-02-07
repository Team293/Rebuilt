package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class IndexerIOTalonFX implements IORefresher, IndexerIO {
    private final TalonFX motor;
    private final BaseStatusSignal motorRps;

    public IndexerIOTalonFX() {
        this.motor = new TalonFX(CanID.INDEXER.getID());
        this.motorRps = motor.getRotorVelocity();
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorRps);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorRps = motorRps.getValueAsDouble();
    }

    @Override
    public void setSpeed(double rps) {
        motor.set(rps);
    }
}