package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        // if (targetHub) {
        //     this.targeting.setTargetingHub();
        // } else {
        //     this.targeting.setTargetingShuttleRight(); // TODO: Allow for left selection as well
        // }
        this.targeting.setTargetingHub();

        addRequirements(shooter, targeting);
    }

    @Override
    public void initialize() {
        this.scoringTimer.restart();

        this.shooter.setDriverRequestingShooting(true);
        this.targeting.setTargetingHub();
    }

    @Override
    public void execute() {
        this.targeting.setTargetingHub();

        if (this.scoringTimer.hasElapsed(3)) {
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
