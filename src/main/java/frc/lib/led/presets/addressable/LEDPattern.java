// Implementation based on Team 4481's LED Controller
// https://github.com/FRC-4481-Team-Rembrandts/4481-led-controller
package frc.lib.led.presets.addressable;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.led.LEDStrip;
public interface LEDPattern {
    void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip);
}
