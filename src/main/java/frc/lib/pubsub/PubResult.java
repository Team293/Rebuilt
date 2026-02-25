package frc.lib.pubsub;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * A class that represents the result of a publish action. Contains the timestamp of when the data was published and the data itself.
 * @param <T> The type of data being published.
 */
public class PubResult<T> {
    private final double timestamp;
    private final T data;

    public PubResult(double timestamp, T data) {
        this.timestamp = timestamp;
        this.data = data;
    }

    public PubResult(T data) {
        this(Timer.getFPGATimestamp(), data);
    }

    public boolean isInTimestamp() {
        return Robot.isTimestampInCurrentTick(this.timestamp);
    }

    public double getTimestamp() {
        return timestamp;
    }

    public T getData() {
        return data;
    }
}
