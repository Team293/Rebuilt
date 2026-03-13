package frc.robot.subsystems.targeting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    static {
        distanceToRPM.put(2.7, 1900.0);
        distanceToHoodAngle.put(2.7, 30.0);

        distanceToRPM.put(3.0, 2000.0);
        distanceToHoodAngle.put(3.0, 30.0);

        distanceToRPM.put(4.0, 2400.0);
        distanceToHoodAngle.put(4.0, 30.0);
    }
}
