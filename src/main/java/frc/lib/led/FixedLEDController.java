package frc.lib.led;

/**
 * A controller for fixed LED strips connected to a PWM port on the RoboRIO.
 * Thin wrapper over PWMLEDController implementation for compatibility purposes.
 */
public class FixedLEDController {
    private final PWMLEDController controller;

    public FixedLEDController(int port) {
        controller = new PWMLEDController(port, PWMLEDController.Mode.FIXED);
    }

    public void switchPreset(String name) {
        controller.switchPreset(name);
    }

    public void registerPreset(LEDPreset preset) {
        controller.registerPreset(preset);
    }
}
