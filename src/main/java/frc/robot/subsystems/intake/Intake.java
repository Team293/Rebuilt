package frc.robot.subsystems.intake;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Intake extends SpikeSystem<IntakeIO.IntakeIOInputs> {
     private static final double BASE_SPEED = 1;//add in accurate numbers when protoype built
     private static final double SPEED_PER_MPS = 0.05;
     private static final double MAX_SPEED = 5.0;

    private final IntakeIOTalonFX intakeIO;
    private final CommandSwerveDrivetrain drivetrain;

    private boolean running = false;

    public Intake(CommandSwerveDrivetrain drivetrain) {
        super("Intake", new IntakeIO.IntakeIOInputs());
        this.drivetrain = drivetrain;
        this.intakeIO = new IntakeIOTalonFX();
    }

    @Override
    public void onPeriodic() {
        if (running) {
            double driveSpeed = Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond);
            double speed = Math.min(BASE_SPEED + driveSpeed * SPEED_PER_MPS, MAX_SPEED); 
            intakeIO.on(speed);
        }
    }

     public void on() {
        running = true;
    }

     public void off() {
        running = false;
        intakeIO.off();
    }

     public void deploy() {
        intakeIO.deploy();
    }

     public void retract() {
        intakeIO.retract();
    }

    @Override
    protected Runnable setupDataRefresher() { //Spikelib 
        return useDataRefresher(intakeIO);
    }
}