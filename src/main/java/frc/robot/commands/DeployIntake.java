package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class DeployIntake extends Command {
    private Intake intake;

    public DeployIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.deployIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
