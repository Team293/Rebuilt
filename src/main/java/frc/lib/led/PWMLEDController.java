package frc.lib.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Consumer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Unified RGB LED controller that can drive either a fixed 12V strip or an
 * individually addressable 5V strip.
 *
 * <p>Use {@link Mode#FIXED} for Spark/PWM-based controllers and
 * {@link Mode#ADDRESSABLE} for addressable LED buffers.
 */
public class PWMLEDController {
    /** Selects which hardware model the controller should use. */
    public enum Mode {
        FIXED,
        ADDRESSABLE
    }

    /**
     * Defines one logical preset name with a fixed implementation and an
     * addressable implementation.
     *
     * <p>The addressable mutator receives a read-only strip list. It can mutate
     * strip state (pattern/colors/duration) but should not modify strip topology.
     */
    public record ModePreset(
            String name,
            LEDPreset fixedPreset,
            Consumer<List<LEDStrip>> addressableMutator
    ) {}

    private final Mode mode;
    private final LEDBackend backend;

    /**
     * Creates a new RGB controller for either fixed or addressable hardware.
     *
     * @param port The PWM port the controller is connected to.
     * @param mode The hardware mode to use.
     */
    public PWMLEDController(int port, Mode mode) {
        this.mode = mode;
        this.backend = switch (mode) {
            case FIXED -> new FixedBackend(port);
            case ADDRESSABLE -> new AddressableBackend(port);
        };
    }

    /** Returns the configured hardware mode. */
    public Mode getMode() {
        return mode;
    }

    /** Returns true when this controller is using addressable LED hardware. */
    public boolean isAddressable() {
        return mode == Mode.ADDRESSABLE;
    }

    /** Returns true when this controller is using fixed LED hardware. */
    public boolean isFixed() {
        return mode == Mode.FIXED;
    }

    /** Starts the output of the controller. */
    public void start() {
        backend.start();
    }

    /** Stops the output of the controller. */
    public void stop() {
        backend.stop();
    }

    /**
     * Updates the controller.
     *
     * <p>For fixed hardware this is a no-op, while addressable hardware pushes
     * the current strip buffers to the RoboRIO output.
     */
    public void update() {
        backend.update();
    }

    /** Registers a preset for fixed LED hardware. */
    public void registerPreset(LEDPreset preset) {
        backend.registerPreset(preset);
    }

    /** Registers a mode-aware preset for either fixed or addressable hardware. */
    public void registerPreset(ModePreset preset) {
        backend.registerPreset(preset);
    }

    /** Switches the active preset for the currently selected hardware mode. */
    public void switchPreset(String name) {
        backend.switchPreset(name);
    }

    /** Adds a strip for addressable LED hardware. */
    public void addStrip(LEDStrip strip) throws DuplicateLEDAssignmentException {
        backend.addStrip(strip);
    }

    /** Adds a strip for addressable LED hardware. */
    public void addStrip(int length, int offset) throws DuplicateLEDAssignmentException {
        backend.addStrip(length, offset);
    }

    /** Gets an addressable strip by index. */
    public LEDStrip getStrip(int index) {
        return backend.getStrip(index);
    }

    /** Removes an addressable strip by index. */
    public void removeStrip(int index) {
        backend.removeStrip(index);
    }

    /** Returns the number of configured strips. */
    public int getStripCount() {
        return backend.getStripCount();
    }

    private interface LEDBackend {
        void start();

        void stop();

        void update();

        void registerPreset(LEDPreset preset);

        void registerPreset(ModePreset preset);

        void switchPreset(String name);

        void addStrip(LEDStrip strip) throws DuplicateLEDAssignmentException;

        void addStrip(int length, int offset) throws DuplicateLEDAssignmentException;

        LEDStrip getStrip(int index);

        void removeStrip(int index);

        int getStripCount();
    }

    private static abstract class BaseBackend implements LEDBackend {
        @Override
        public void registerPreset(LEDPreset preset) {
            throw unsupported("registerPreset");
        }

        @Override
        public void registerPreset(ModePreset preset) {
            throw unsupported("registerPreset");
        }

        @Override
        public void switchPreset(String name) {
            throw unsupported("switchPreset");
        }

        @Override
        public void addStrip(LEDStrip strip) throws DuplicateLEDAssignmentException {
            throw unsupported("addStrip");
        }

        @Override
        public void addStrip(int length, int offset) throws DuplicateLEDAssignmentException {
            throw unsupported("addStrip");
        }

        @Override
        public LEDStrip getStrip(int index) {
            throw unsupported("getStrip");
        }

        @Override
        public void removeStrip(int index) {
            throw unsupported("removeStrip");
        }

        @Override
        public int getStripCount() {
            return 0;
        }

        protected UnsupportedOperationException unsupported(String operation) {
            return new UnsupportedOperationException(
                    operation + " is only supported for addressable or fixed hardware as applicable"
            );
        }
    }

    private static final class FixedBackend extends BaseBackend {
        private final Spark controller;
        private final Map<String, LEDPreset> presets = new HashMap<>();

        private FixedBackend(int port) {
            controller = new Spark(port);
        }

        @Override
        public void start() {
            // Fixed PWM strips are driven directly by setting the controller output.
        }

        @Override
        public void stop() {
            controller.set(0);
        }

        @Override
        public void update() {
            // Fixed strips do not need a periodic buffer push.
        }

        @Override
        public void registerPreset(LEDPreset preset) {
            presets.put(preset.name(), preset);
        }

        @Override
        public void registerPreset(ModePreset preset) {
            if (preset.fixedPreset() != null) {
                registerPreset(preset.fixedPreset());
            }
        }

        @Override
        public void switchPreset(String name) {
            LEDPreset targetPreset = presets.get(name);
            if (targetPreset != null) {
                controller.set(targetPreset.value());
            } else {
                System.err.println("Invalid preset name, did you register it?");
            }
        }
    }

    private static final class AddressableBackend extends BaseBackend {
        private final AddressableLED controller;
        private AddressableLEDBuffer buffer;
        private final ArrayList<LEDStrip> strips = new ArrayList<>();
        private final Map<String, Consumer<List<LEDStrip>>> presets = new HashMap<>();

        private AddressableBackend(int port) {
            controller = new AddressableLED(port);
            buffer = new AddressableLEDBuffer(0);
            controller.setLength(0);
            controller.setData(buffer);
        }

        @Override
        public void start() {
            controller.start();
        }

        @Override
        public void stop() {
            controller.stop();
        }

        @Override
        public void update() {
            for (LEDStrip strip : strips) {
                strip.getPattern().updateBuffer(buffer, strip);
            }

            controller.setData(buffer);
        }

        @Override
        public void addStrip(LEDStrip strip) throws DuplicateLEDAssignmentException {
            int index = getNewStripIndex(strip.getLength(), strip.getOffset());

            if (index == strips.size()) {
                strips.add(strip);
            } else {
                strips.add(index, strip);
            }

            updateTotalStripLength();
        }

        @Override
        public void addStrip(int length, int offset) throws DuplicateLEDAssignmentException {
            addStrip(new LEDStrip(length, offset));
        }

        @Override
        public void registerPreset(ModePreset preset) {
            if (preset.addressableMutator() != null) {
                presets.put(preset.name(), preset.addressableMutator());
            }
        }

        @Override
        public void registerPreset(LEDPreset preset) {
            // Fixed presets are ignored in addressable mode.
        }

        @Override
        public void switchPreset(String name) {
            Consumer<List<LEDStrip>> stripMutator = presets.get(name);
            if (stripMutator == null) {
                System.err.println("Invalid preset name, did you register it?");
                return;
            }

            try {
                stripMutator.accept(Collections.unmodifiableList(strips));
            } catch (RuntimeException ex) {
                throw new IllegalStateException(
                        "Failed to apply addressable preset '" + name + "'",
                        ex
                );
            }
        }

        @Override
        public LEDStrip getStrip(int index) {
            return strips.get(index);
        }

        @Override
        public void removeStrip(int index) {
            strips.remove(index);
            updateTotalStripLength();
        }

        @Override
        public int getStripCount() {
            return strips.size();
        }

        private void updateTotalStripLength() {
            int totalLength = 0;
            if (!strips.isEmpty()) {
                LEDStrip lastStrip = strips.get(strips.size() - 1);
                totalLength = lastStrip.getOffset() + lastStrip.getLength();
            }

            buffer = new AddressableLEDBuffer(totalLength);
            controller.setLength(totalLength);
            controller.setData(buffer);
        }

        private int getNewStripIndex(int length, int offset) {
            int newStripEnd = offset + length;

            for (int i = 0; i < strips.size(); i++) {
                LEDStrip strip = strips.get(i);
                int stripStart = strip.getOffset();
                int stripEnd = stripStart + strip.getLength();

                if (offset < stripEnd && newStripEnd > stripStart) {
                    throw new DuplicateLEDAssignmentException(
                            "LED strip overlaps with existing strip at index " + i
                    );
                }
            }

            for (int i = 0; i < strips.size(); i++) {
                if (strips.get(i).getOffset() > offset) {
                    return i;
                }
            }

            return strips.size();
        }
    }
}

