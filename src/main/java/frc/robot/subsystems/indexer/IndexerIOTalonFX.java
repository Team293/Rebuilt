package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;

public class IndexerIOTalonFX implements IORefresher, IndexerIO {
    private final TalonFX motor;             // Motor object
    private final BaseStatusSignal motorRps; // Rotations per second

    public IndexerIOTalonFX() {
        this.motor = new TalonFX(CanID.INDEXER_MOTOR.getID());
        this.motorRps = motor.getRotorVelocity();
    }

    // Refresh all signals
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