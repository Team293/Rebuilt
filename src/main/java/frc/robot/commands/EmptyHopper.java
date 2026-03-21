package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.targeting.Targeting;

public class EmptyHopper extends Command {
    private Shooter shooter;
    private Targeting targeting;

    private double scoringTime;
    private Timer scoringTimer = new Timer();

    public EmptyHopper(Shooter shooter, Targeting targeting, double forTime) {
        this.shooter = shooter;
        this.targeting = targeting;

        this.scoringTime = forTime;

        this.shooter.setDriverRequestingShooting(true);
    }

    public void initialize() {
        this.scoringTimer.restart();
        this.targeting.setTargetingHub();
    }

    @Override
    public boolean isFinished() {
        return this.scoringTimer.hasElapsed(scoringTime);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        this.shooter.setDriverRequestingShooting(false);
    }
}
