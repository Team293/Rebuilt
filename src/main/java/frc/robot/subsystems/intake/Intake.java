package frc.robot.subsystems.intake;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Intake extends SpikeSystem<IntakeIO.IntakeIOInputs> {
    private static final double kIntakeSpeedRatio = 10.0; // scaling factor for intake speed
    private static final double kBaseIntakeSpeed = 5.0; // base speed for intake motor (rps)

    private final IntakeIOTalonFX intakeIO = new IntakeIOTalonFX();
    private final CommandSwerveDrivetrain drive;

    public Intake(CommandSwerveDrivetrain drive) {
        super("Intake", new IntakeIO.IntakeIOInputs());
        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        double driveSpeed = drive.getState().Speeds.vxMetersPerSecond;
        // make intake motorRps correlated to drive speed
        double intakeSpeed = kBaseIntakeSpeed + (Math.abs(driveSpeed) / kIntakeSpeedRatio);

        intakeIO.setSpeed(intakeSpeed);
    }

    @Override
    protected Runnable setupDataRefresher() {
        return useAsyncDataRefresher(intakeIO);
    }
}
