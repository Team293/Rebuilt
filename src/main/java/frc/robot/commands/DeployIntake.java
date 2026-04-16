package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class DeployIntake extends Command {
    private Intake intake;
    private Timer timer = new Timer();

    public DeployIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setDeployServo(1.0);
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.75)) {
            intake.setDeployServo(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(1.5)) {
            return true;
        }
        return false;
    }
    
}
