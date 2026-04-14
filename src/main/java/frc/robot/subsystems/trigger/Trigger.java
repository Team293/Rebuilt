package frc.robot.subsystems.trigger;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class Trigger extends SpikeSystem<TriggerIO.TriggerIOInputs> {
    private static final int PROXIMITY_SENSOR_CHANNEL = 2; // DIO channel for the proximity sensor

    private final static double TRIGGER_SPEED = 30.0; // Rotations per second

    private final Shooter shooter;
    private final Turret turret;

    private TriggerIO triggerIO;

    private boolean reverseTrigger = false;

    public Trigger(Shooter shooter, Turret turret) {
        super("Trigger", new TriggerIOInputsAutoLogged());

        this.shooter = shooter;
        this.turret = turret;
    }

    // Activate motor if proximity sensor detects a ball in the indexer
    @Override
    public void onPeriodic() {
        if (this.reverseTrigger) {
            triggerIO.setSpeed(-TRIGGER_SPEED);
        } else if (mechanismReadyForBalls()) {
            // run the indexer if the mechanisms are ready for balls
            // run it regardless of ball in indexer, so that it can feed a ball in if there is one queued up
            triggerIO.setSpeed(TRIGGER_SPEED);
        } else {
            // stop the indexer if the mechanisms aren't ready and we have a ball queued
            triggerIO.setSpeed(0.0);
        }
    }

    public void setReverseTrigger(boolean reverse) {
        this.reverseTrigger = reverse;
    }

    public boolean getReverseTrigger() {
        return this.reverseTrigger;
    }

    /**
     * Determines if the shooter and turret are ready to receive a ball.
     * @return true if the shooter is at target RPS, the turret is at target angle, and the driver is requesting to shoot, false otherwise
     */
    private boolean mechanismReadyForBalls() {
        if (shooter.isActuatingHoodAndLaunching()) {
            // check if turret is at target angle 
            boolean turretReady = turret.isAtTargetAngle();

            if (!turretReady) {
                return false;
            }
            return true;
        }
        return false;
    }

    /**
     * Determines if the indexer needs a ball fed into it.
     * True if the mechanisms need a ball and there isn't one already queued up, false otherwise.
     * @return true if the indexer needs a ball, false otherwise
     */
    public boolean needsFeeding() {
        return mechanismReadyForBalls();
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.triggerIO = new TriggerIOTalonFX(PROXIMITY_SENSOR_CHANNEL);
        return useAsyncDataRefresher(triggerIO);
    }
}