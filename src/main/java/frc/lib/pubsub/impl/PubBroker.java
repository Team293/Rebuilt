package frc.lib.pubsub.impl;

import frc.lib.pubsub.PubResult;
import frc.lib.pubsub.PubTopic;
import frc.lib.pubsub.SimpleDataPub;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Backing broker for the publishing system. Each broker instance is associated with a specific type of data (T) and stores the latest published value along with its timestamp.
 * @param <T> The type of data being published and subscribed to.
 */
public class PubBroker<T> implements SimpleDataPub<T> {
    private static final Map<PubTopic, PubBroker<?>> brokers = new ConcurrentHashMap<>();

    // reference for 'T', atomic to ensure thread safety when reading the reference
    private final AtomicReference<PubResult<T>> ref = new AtomicReference<>();

    private PubBroker(PubTopic topic) {
        brokers.put(topic, this);
    }

    /**
     * Get an existing broker for the specified type or create a new one if it doesn't exist.
     * @return A PubBroker instance for the specified type.
     * @param <N> The type of data the broker should handle
     */
    public static <N> PubBroker<N> getOrCreate(PubTopic topic) {
        // search through the set of brokers for one that matches the requested type
        PubBroker<?> broker = brokers.get(topic);
        if (broker != null) {
            try {
                // attempt to cast the broker to the requested type
                return (PubBroker<N>) broker;
            } catch (ClassCastException e) {
                // if the cast fails, it means the existing broker is for a different type; log an error and create a new broker
                System.err.println("Error: Existing broker for topic " + topic + " is of a different type. Creating new broker.");
                brokers.remove(topic); // remove the existing broker to avoid future conflicts
            }
        }

        // not found; create new broker
        return new PubBroker<>(topic);
    }

    @Override
    public void publish(T data, double timestamp) {
        ref.set(new PubResult<>(timestamp, data));
    }

    @Override
    public Optional<PubResult<T>> poll() {
        PubResult<T> result = ref.get();
        return Optional.ofNullable(result);
    }
}
