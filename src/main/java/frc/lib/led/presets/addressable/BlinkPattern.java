// Implementation based on Team 4481's LED Controller
// https://github.com/FRC-4481-Team-Rembrandts/4481-led-controller
package frc.lib.led.presets.addressable;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.led.Color;
import frc.lib.led.LEDStrip;
import frc.lib.CountingDelay;

/**
 * A blinking pattern that can be applied to an LED strip.
 * This pattern will alternate between the primary and secondary colors.
 * @see LEDStrip
 */
public class BlinkPattern implements LEDPattern {
    private final CountingDelay blinkDelay = new CountingDelay();
    private boolean blinkPrimary = true;

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
        Color.HSV primaryColor = strip.getPrimaryColor().getHSV();
        Color.HSV secondaryColor = strip.getSecondaryColor().getHSV();
        double blinkDuration = strip.getPatternDuration();

        // Switching states should be done twice per duration
        if (blinkDelay.delay(blinkDuration / 2)) {
            blinkPrimary = !blinkPrimary;
            blinkDelay.reset();
        }

        Color.HSV color = blinkPrimary ? primaryColor : secondaryColor;

        // Set the LEDs to the color if blinkOn is true, otherwise set them to black
        for (int i = 0; i < length; i++) {
            buffer.setHSV(
                    offset + i,
                    color.hue(),
                    color.saturation(),
                    color.value()
            );
        }
    }
}
