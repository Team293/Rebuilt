package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SetIntakeState extends Command {
    private Intake intake;
    private boolean enabled;

    public SetIntakeState(Intake intake, boolean enabled) {
        this.intake = intake;
        this.enabled = enabled;
        if (this.enabled) {
            this.intake.enable();
        } else {
            this.intake.disable();
        }
    }

    @Override
    public void initialize() {
        if (this.enabled) {
            this.intake.enable();
        } else {
            this.intake.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
