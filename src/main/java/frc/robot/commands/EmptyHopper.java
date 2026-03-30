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

    public EmptyHopper(Shooter shooter, Targeting targeting, double forTime, boolean targetHub) {
        this.shooter = shooter;
        this.targeting = targeting;

        this.scoringTime = forTime;

        this.shooter.setDriverRequestingShooting(true);
        
        if (targetHub) {
            this.targeting.setTargetingHub();
        } else {
            this.targeting.setTargetingShuttleRight(); // TODO: Allow for left selection as well
        }
    }

    @Override
    public void initialize() {
        this.scoringTimer.restart();
        this.targeting.setTargetingHub();
    }

    @Override
    public void execute() {
        this.targeting.setTargetingHub();

        if (this.scoringTimer.hasElapsed(3.0)) {
            this.shooter.setRequestingWithForce(true);
        }
    }

    @Override
    public boolean isFinished() {
        return this.scoringTimer.hasElapsed(scoringTime);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        this.shooter.setDriverRequestingShooting(false);
        this.shooter.setRequestingWithForce(false);
    }
}
