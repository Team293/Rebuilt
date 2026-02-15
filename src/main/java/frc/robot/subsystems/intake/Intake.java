package frc.robot.subsystems.intake;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Intake extends SpikeSystem<IntakeIO.IntakeIOInputs> {
     private static final double BASE_SPEED = 1; // Rotation Per Second  
     private static final double SPEED_PER_MPS = 0.05; //speed per m/s of drive velocity
     private static final double MAX_SPEED = 5.0; //  speed cap

    private final IntakeIOTalonFX intakeIO;
    private final CommandSwerveDrivetrain drivetrain;

    private boolean running = false;

    public Intake(CommandSwerveDrivetrain drivetrain) {
        super("Intake", new IntakeIO.IntakeIOInputs());
        this.drivetrain = drivetrain;
        this.intakeIO = new IntakeIOTalonFX();
    }

     // Scales intake speed dependent of drivetrain speed 
     @Override
    public void onPeriodic() {
        if (running) {
             double driveSpeed = Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond);
             double speed = Math.min(BASE_SPEED + driveSpeed * SPEED_PER_MPS, MAX_SPEED);
            intakeIO.on(speed);
        }
    }

     // Enables the intake 
     public void on() {
        running = true;
    }

     // Disables the intake 
     public void off() {
        running = false;
        intakeIO.off();
    }

     // Deploys the OTB intake 
     public void deploy() {
        intakeIO.deploy();
    }

     // Retracts the intake back to its stored position
     public void retract() {
        intakeIO.retract();
    }

    @Override
    protected Runnable setupDataRefresher() {
        return useAsyncDataRefresher(intakeIO);
    }
}
