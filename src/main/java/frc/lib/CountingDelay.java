package frc.lib;

import edu.wpi.first.wpilibj.Timer;

public class CountingDelay {
    private boolean lock;
    private double startTimeStamp;

    public CountingDelay() {
        reset();
    }

    public boolean delay(double time){
        if (!lock) {
            startTimeStamp = Timer.getFPGATimestamp();
            lock = true;
        }
        return Timer.getFPGATimestamp() - startTimeStamp >= time;
    }
    public void reset(){
        lock = false;
    }
}
