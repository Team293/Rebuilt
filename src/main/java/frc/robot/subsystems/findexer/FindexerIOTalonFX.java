package frc.robot.subsystems.findexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CanID;

public class FindexerIOTalonFX implements FindexerIO {
    private final TalonFX motor;

    private final StatusSignal<AngularVelocity> motorRps; // Rotations per second

    public FindexerIOTalonFX() {
        this.motor = new TalonFX(CanID.FINDEXER_MOTOR.getID());
        this.motorRps = motor.getVelocity();

        this.motor.optimizeBusUtilization();
    }

    @Override
    public void setSpeed(double rps) {
        this.motor.set(rps);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorRps);
    }

    @Override
    public void updateInputs(FindexerIOInputs inputs) {
        inputs.motorRps = motorRps.getValueAsDouble();
    }
}
