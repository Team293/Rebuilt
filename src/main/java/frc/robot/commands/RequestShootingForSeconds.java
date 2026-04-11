package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RequestShootingForSeconds extends Command {
    private Shooter shooter;
    private boolean enabled;

    private Timer shootingTimer = new Timer();
    private double shootingTime = 0;

    public RequestShootingForSeconds(Shooter shooter, boolean enabled, double shootingTime) {
        this.shooter = shooter;
        this.enabled = enabled;
        this.shootingTime = shootingTime;
        this.shootingTimer.restart();
    }


    @Override
    public void initialize() {
        this.shooter.setActuateHoodAndLaunch(enabled);
        this.shootingTimer.restart();
        System.out.println("RequestShootingForSeconds initialized with enabled = " + enabled + " and shootingTime = " + shootingTime);
    }

    @Override
    public void execute() {
        this.shooter.setActuateHoodAndLaunch(enabled);
        this.shooter.setDriverSpinUpFlywheel(true);
        System.out.println("RequestShootingForSeconds executing with enabled = " + enabled);
    }

    @Override
    public boolean isFinished() {
        return this.shootingTimer.hasElapsed(shootingTime);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.setActuateHoodAndLaunch(false);
        this.shooter.setDriverSpinUpFlywheel(false);
    }
}
