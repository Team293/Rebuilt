package frc.lib.led;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDController {
    private static final Map<String, LEDPreset> presets = new HashMap<>();
    private Spark controller;

    public LEDController(int port) {
        controller = new Spark(port);
    }

    public void switchPreset(String name) {
        LEDPreset targetPreset = presets.get(name);
        if (targetPreset != null) {
            controller.set(targetPreset.value());
        } else {
            System.err.println("Invalid preset name, did you register it?");
        }
    }

    public void registerPreset(LEDPreset preset) {
        presets.put(preset.name(), preset);
    }
}