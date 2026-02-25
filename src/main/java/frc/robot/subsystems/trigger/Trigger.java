package frc.robot.subsystems.trigger;

import frc.lib.subsystem.SpikeSystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Trigger extends SpikeSystem<TriggerIO.TriggerIOInputs> {
    private final static double triggerSpeed = 10.0; // Rotations per second

    private TriggerIO triggerIO;
    private DigitalInput proximitySensor;

    public Trigger(int channel) {
        super("Trigger", new TriggerIO.TriggerIOInputs());
        proximitySensor = new DigitalInput(channel);
    }

    // Activate motor if proximity sensor detects a ball in the trigger
    @Override
    public void onPeriodic() {
        if (proximitySensor.get()) {
            triggerIO.setSpeed(0.0);
        } else {
            triggerIO.setSpeed(triggerSpeed);
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.triggerIO = new TriggerIOTalonFX();
        return useAsyncDataRefresher(triggerIO);
    }
}