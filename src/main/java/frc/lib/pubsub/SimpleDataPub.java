package frc.lib.pubsub;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.pubsub.impl.PubBroker;

import java.util.Optional;

/**
 * A simple publish-subscribe interface for sharing data between subsystems without direct references.
 * @param <T> The type of data being published and subscribed to.
 */
public interface SimpleDataPub<T> {
    /**
     * Publish data to a specific topic. Subsystems can poll.
     * @param data The data to publish
     */
    default void publish(T data) {
        publish(data, Timer.getFPGATimestamp());
    }

    /**
     * Publish data to a specific topic. Subsystems can poll.
     * @param timestamp The timestamp of when the data is published, in seconds. This can be used by subscribers to determine if the data is fresh or stale.
     * @param data The data to publish
     */
    void publish(T data, double timestamp);

    /**
     * Poll for the latest data on a specific topic.
     * @return An Optional containing the latest data if available, or empty if no data has been published yet.
     */
    Optional<PubResult<T>> poll();
}
