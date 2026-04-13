// Implementation based on Team 4481's LED Controller
// https://github.com/FRC-4481-Team-Rembrandts/4481-led-controller
package frc.lib.led.presets.addressable;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.led.Color;
import frc.lib.led.LEDStrip;

/**
 * A solid color pattern that can be applied to an LED strip.
 * @see LEDStrip
 */
public class SolidColorPattern implements LEDPattern {
    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip  The strip to update the buffer with.
     */
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        int offset = strip.getOffset();
        int length = strip.getLength();
        Color.HSV color = strip.getPrimaryColor().getHSV();

        for (int i = 0; i < length; i++) {
            buffer.setHSV(offset + i, color.hue(), color.saturation(), color.value());
        }
    }
}