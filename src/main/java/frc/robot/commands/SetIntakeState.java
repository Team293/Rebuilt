package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SetIntakeState extends Command {
    private Intake intake;
    private boolean enabled;

    public SetIntakeState(Intake intake, boolean enabled) {
        this.intake = intake;
        this.enabled = enabled;
    }

    @Override
    public void initialize() {
        if (this.enabled) {
            this.intake.disable();
        } else {
            this.intake.enable();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
