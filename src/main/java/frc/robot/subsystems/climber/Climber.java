package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystem.SpikeSystem;

public class Climber extends SpikeSystem<ClimberIO.ClimberIOInputs> {
    private ClimberIO climberIO;

    private CommandXboxController operatorController; // Assuming the operator controller is on port 1

    public Climber(CommandXboxController controller) {
        super("Climber", new ClimberIO.ClimberIOInputs());
        this.climberIO = new ClimberIOTalonFX();
        this.operatorController = controller;
    }

    @Override
    public void onPeriodic() {
        double climberVel = operatorController.getLeftY(); // Get the Y-axis value of the left stick for controlling the climber
        if (Math.abs(climberVel) < 0.1) { // Deadband to prevent accidental movement
            climberVel = 0.0;
        } else {
            setClimberVelocity(climberVel * 5.0); // Scale the joystick input to a reasonable speed (5 RPS max)
        }
    }

    public void setClimberVelocity(double velocityRotationsPerSecond) {
        climberIO.setVelocity(velocityRotationsPerSecond);
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.climberIO = new ClimberIOTalonFX();
        return useAsyncDataRefresher(climberIO);
    }   
}
