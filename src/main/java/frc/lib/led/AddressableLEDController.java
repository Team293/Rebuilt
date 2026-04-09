// Implementation based on Team 4481's LED Controller
// https://github.com/FRC-4481-Team-Rembrandts/4481-led-controller
package frc.lib.led;

/**
 * A controller for individually addressable LED strips connected to a PWM port on the RoboRIO.
 * Thin wrapper over PWMLEDController implementation for compatibility purposes.
 */
public class AddressableLEDController {
    private final PWMLEDController controller;

    /**
     * Creates a new LED controller.
     * @param port The PWM port the LED controller is connected to.
     */
    public AddressableLEDController(int port) {
        controller = new PWMLEDController(port, PWMLEDController.Mode.ADDRESSABLE);
    }

    /**
     * Adds an LED strip to the controller.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * If a new strip is added between two existing strips,
     * all strips after the new strip will have their index shifted.
     * But the lengths and offsets will be unaffected.
     * @param strip The LED strip to be added.
     * @throws DuplicateLEDAssignmentException If the new strip overlaps with an existing strip.
     */
    public void addStrip(LEDStrip strip) throws DuplicateLEDAssignmentException {
        controller.addStrip(strip);
    }

    /**
     * Adds a new LED strip to the controller with a certain length and offset from the start.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * If a new strip is added between two existing strips,
     * all strips after the new strip will have their index shifted.
     * But the lengths and offsets will be unaffected.
     * @param length The length of the strip in LEDs.
     * @param offset The offset to the start of the strip in LEDs.
     * @throws DuplicateLEDAssignmentException If the new strip overlaps with an existing strip.
     */
    void addStrip(int length, int offset) throws DuplicateLEDAssignmentException {
        controller.addStrip(length, offset);
    }

    /**
     * Gets the LED strip at a certain index.
     * @param index The index of the strip.
     * @return The LED strip at the index.
     */
    public LEDStrip getStrip(int index) {
        return controller.getStrip(index);
    }

    /**
     * Removes the LED strip at a certain index.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * If the strip that gets removed is not the last strip,
     * all strips after the removed strip will have their index shifted.
     * @param index The index of the strip to remove.
     */
    public void removeStrip(int index) {
        controller.removeStrip(index);
    }

    /**
     * Starts the output of the LED controller.
     * The output writes continuously.
     */
    public void start() {
        controller.start();
    }

    /**
     * Stops the output of the LED controller.
     */
    public void stop() {
        controller.stop();
    }

    /**
     * Updates the LED strips.
     * This should be called in the loop.
     */
    public void updateStrips() {
        controller.update();
    }
}
