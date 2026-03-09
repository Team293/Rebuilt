package frc.robot.subsystems.intake;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Intake extends SpikeSystem<IntakeIO.IntakeIOInputs> {
    private static final double DEPLOY_SPEED = 1.0; // Speed in Rotations Per Second to deploy the intake
    private static final double RETRACT_SPEED = -1.0; // Speed in Rotations Per Second to retract the intake
    private static final double DEPLOY_CURRENT_THRESHOLD = 10.0; // Amp limit for the deploy motor. Watches
                                                                 // for a resistance to the motor to see when it's
                                                                 // deployed
    private static final double BASE_SPEED_INTAKE = 1; // Rotation Per Second
    private static final double SPEED_PER_MPS = 0.05; // Speed added to the base speed per m/s of drive velocity
    private static final double MAX_SPEED = 5.0; // speed cap, max speed of the intake in RPS

    public enum IntakeState {
        DEPLOYED, RETRACTED, DEPLOYING, RETRACTING
    }

    private IntakeIO intakeIO;
    private final CommandSwerveDrivetrain drivetrain;

    private boolean running = false; // True if the intake is running, False otherwise

    // Intake constructor
    public Intake(CommandSwerveDrivetrain drivetrain) {
        super("Intake", new IntakeIOInputsAutoLogged());
        this.drivetrain = drivetrain;
        enable();
    }

    // Intake periodic function
    // Increases intake speed when driving faster
    @Override
    public void onPeriodic() {
        // Only act on the intake motor if the intake is running
        if (running) {
            // switch (intakeIO.getIntakeState()) {
            //     case DEPLOYING: // Intake is deploying
            //         doDeployingState();
            //         break;
            //     case RETRACTING: // Intake is retracting
            //         doRetractingState();
            //         break;
            //     case DEPLOYED: // Intake is fully deployed
            //         doDeployedState();
            //         break;
            //     case RETRACTED: // Intake is fully retracted
            //         doRetractedState();
            //         break;
            //     default:
            //         // Error, unknown state!
            //         // Turn off the deploy and intake motors!
            //         intakeIO.setDeploySpeed(0.0);
            //         intakeIO.setIntakeSpeed(0.0);
            //         // Log unknown state
            //         break;
            // }
            doDeployedState();
        }
    }

    // Run the DEPLOYING state periodic actions
    private void doDeployingState() {
        // Make sure the intake motor is OFF
        intakeIO.setIntakeSpeed(0.0);
        // Set intake speed to deploy
        intakeIO.setDeploySpeed(DEPLOY_SPEED);
        // Check to see if the intake has fully deployed
        if (intakeIO.getDeployMotorCurrent() > DEPLOY_CURRENT_THRESHOLD) { // If the deploy motor current exceeds the
                                                                           // threshold, we can assume it's fully
                                                                           // deployed
            intakeIO.setDeploySpeed(0.0); // Disable the deploy motor
            intakeIO.setIntakeState(IntakeState.DEPLOYED); // Mark as deployed
        }
    }

    // Run the DEPLOYED state periodic actions
    private void doDeployedState() {
        // Get the absolute velocity of the entire robot
        double robotAbsoluteVelocity = Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond);

        // Adjust the intake speed, increase the intake as the robot moves faster
        // Cap at MAX_SPEED
        double speed = Math.min(BASE_SPEED_INTAKE + robotAbsoluteVelocity * SPEED_PER_MPS, MAX_SPEED);

        // Update the intakeIO on speed
        intakeIO.setIntakeSpeed(speed);
    }

    // Run the RETRACTING state periodic actions
    private void doRetractingState() {
        // Make sure the intake motor is OFF
        intakeIO.setIntakeSpeed(0.0);
        // Set the intake speed to retract
        intakeIO.setDeploySpeed(RETRACT_SPEED);
        // Check to see if the intake has fully retraced
        if (intakeIO.getDeployMotorCurrent() > DEPLOY_CURRENT_THRESHOLD) { // If the deploy motor current exceeds the
                                                                           // threshold, we can assume it's fully
                                                                           // retracted
            intakeIO.setDeploySpeed(0.0); // Disable the deploy motor
            intakeIO.setIntakeState(IntakeState.RETRACTED); // Mark as retracted
        }
    }

    // Run the RETRACTED state periodic actions
    private void doRetractedState() {
        // Intake is fully deployed or retracted
        // Make sure intake motor is OFF
        intakeIO.setIntakeSpeed(0.0);

        // Make sure deploy motor is OFF
        intakeIO.setDeploySpeed(0.0);
    }

    // Enables the intake
    public void enable() {
        running = true;
    }

    // Disables the intake
    public void disable() {
        running = false;

        // Turn off the intake motor
        intakeIO.setIntakeSpeed(0.0);
        intakeIO.setDeploySpeed(0.0);
    }

    // Deploys the over the bumper intake
    public void deploy() {
        if (intakeIO.getIntakeState() != IntakeState.DEPLOYED) { // Only try to deploy if we aren't already deployed
            intakeIO.setIntakeState(IntakeState.DEPLOYING); // Mark as deploying
        }
    }

    // Retracts the intake back to its stored position
    public void retract() {
        if (intakeIO.getIntakeState() != IntakeState.RETRACTED) { // Only try to retract if we aren't already retracted
            intakeIO.setIntakeState(IntakeState.RETRACTING); // Mark as retracting
        }
    }

    // Returns true if the intake is fully deployed, false otherwise
    public boolean isDeployed() {
        return intakeIO.getIntakeState() == IntakeState.DEPLOYED;
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.intakeIO = new IntakeIOTalonFX();
        return useAsyncDataRefresher(intakeIO);
    }

    // Toggles the intake between deployed and retracted states
    public void toggleIntake() {
        if (intakeIO.getIntakeState() == IntakeState.DEPLOYED || intakeIO.getIntakeState() == IntakeState.DEPLOYING) {
            retract();
        } else {
            deploy();
        }
    }
}
