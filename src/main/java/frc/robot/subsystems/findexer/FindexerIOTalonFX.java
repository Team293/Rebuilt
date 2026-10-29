package frc.robot.subsystems.findexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CanID;
import frc.robot.MotorCurrentLimits;

public class FindexerIOTalonFX implements FindexerIO {
    private final TalonFX motor;

    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
    private final StatusSignal<AngularVelocity> motorRps; // Rotations per second

    public FindexerIOTalonFX() {
        this.motor = new TalonFX(CanID.FINDEXER_MOTOR.getID());
        this.motorRps = motor.getVelocity();
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.CurrentLimits = MotorCurrentLimits.findexer();

        Slot0Configs slot0 = config.Slot0;

        slot0.kP = 10;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 2;
        slot0.kV = 0.1;

        config.Slot0 = slot0;

        this.motor.getConfigurator().apply(config);

        this.motor.optimizeBusUtilization();
    }

    /**
     * Set the speed of the findexer motor in rotations per second.
     * @param rps The desired speed in rotations per second
     */
    @Override
    public void setSpeed(double rps) {
        this.velocityControl.Velocity = rps;
        this.motor.setControl(velocityControl);
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
