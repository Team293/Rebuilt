package frc.robot.subsystems.intake;

import frc.lib.subsystem.SpikeSystem;

public class Intake extends SpikeSystem<IntakeIO.IntakeIOInputs> {

    private IntakeIO intakeIO;

    private boolean running = false; // True if the intake is running, False otherwise
    private boolean forward = true;

    // Intake constructor
    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());
    }

    // Intake periodic function
    // Increases intake speed when driving faster
    @Override
    public void onPeriodic() {
        // Only act on the intake motor if the intake is running
        if (running) {
            doDeployedState();
        } else {
            doRetractedState();
        }
    }

    // Run the DEPLOYED state periodic actions
    private void doDeployedState() {
        // Update the intakeIO on speed
        if (forward == false) {
            intakeIO.setIntakeSpeed(-70);
            return;
        }
        intakeIO.setIntakeSpeed(70);
    }

    // Run the RETRACTED state periodic actions
    private void doRetractedState() {
        // Intake is fully deployed or retracted
        // Make sure intake motor is OFF
        intakeIO.setIntakeSpeed(0.0);
    }

    // Enables the intake
    public void enable() {
        running = true;
    }

    public void switchDirection() {
        forward = !forward;
    }

    // Disables the intake
    public void disable() {
        running = false;

        // Turn off the intake motor
        intakeIO.setIntakeSpeed(0.0);
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.intakeIO = new IntakeIOTalonFX();
        return useAsyncDataRefresher(intakeIO);
    }

    // Toggles the intake between deployed and retracted states
    public void toggle() {
        this.running = !this.running;
    }

    public void setDeployServo(double position) {
        this.intakeIO.setDeployServo(position);
    }
}
