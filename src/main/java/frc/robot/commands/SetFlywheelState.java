package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SetFlywheelState extends Command {
    private Shooter shooter;
    private boolean enabled;

    public SetFlywheelState(Shooter shooter, boolean enabled) {
        this.shooter = shooter;
        this.enabled = enabled;
    }

    @Override
    public void initialize() {
        this.shooter.setDriverSpinUpFlywheel(enabled);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
