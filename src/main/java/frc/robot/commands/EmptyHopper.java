package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class EmptyHopper extends Command {
    private Shooter shooter;

    private double scoringTime;
    private Timer scoringTimer = new Timer();

    public EmptyHopper(Shooter shooter, double forTime) {
        this.scoringTime = forTime;
        this.shooter = shooter;

        this.shooter.setDriverRequestingShooting(true);
    }

    public void initialize() {
        this.scoringTimer.restart();
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
