package frc.robot.subsystems.findexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CanID;

public class FindexerIOTalonFX implements FindexerIO {
    private final TalonFX motor;

    private final VelocityVoltage velocityCommand = new VelocityVoltage(0);
    private final StatusSignal<AngularVelocity> motorRps; // Rotations per second

    public FindexerIOTalonFX() {
        this.motor = new TalonFX(CanID.FINDEXER_MOTOR.getID());
        this.motorRps = motor.getVelocity();
        Slot0Configs config = new Slot0Configs();
        config.kP = 0.5;
        config.kI = 0.0;
        config.kD = 0.0;
        
        config.kS = 0.0;
        this.motor.getConfigurator().apply(config);

        this.motor.optimizeBusUtilization();
    }

    /**
     * Set the speed of the findexer motor in rotations per second.
     * @param rps The desired speed in rotations per second
     */
    @Override
    public void setSpeed(double rps) {
        this.velocityCommand.withVelocity(rps);

        this.motor.setControl(velocityCommand);
    }

    /**
     * Refresh all status signals. Automatically called periodically
     */
    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(motorRps);
    }

    /**
     * Updates the inputs 
     * Called automatically
     */
    @Override
    public void updateInputs(FindexerIOInputs inputs) {
        inputs.motorRps = motorRps.getValueAsDouble();
    }
}
