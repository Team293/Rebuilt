// Implementation based on Team 4481's LED Controller
// https://github.com/FRC-4481-Team-Rembrandts/4481-led-controller
package frc.lib.led;

public class InvalidColorException extends RuntimeException {
    public InvalidColorException(String message) {
        super(message);
    }
}
