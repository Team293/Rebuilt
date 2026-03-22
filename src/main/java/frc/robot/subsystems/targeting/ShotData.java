package frc.robot.subsystems.targeting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    static {
        distanceToRPM.put(3.14, 2200.0);
        distanceToHoodAngle.put(3.14, 30.0);

        distanceToRPM.put(4.7, 2550.0);
        distanceToHoodAngle.put(4.7, 36.0);

        distanceToRPM.put(1.5, 1900.0);
        distanceToHoodAngle.put(1.5, 15.0);

        distanceToRPM.put(2.5, 2300.0);
        distanceToHoodAngle.put(2.5, 25.0);

        // distanceToRPM.put(2.96, 2300.0);
        // distanceToHoodAngle.put(2.96, 30.0);

        // distanceToRPM.put(2.7, 1900.0);
        // distanceToHoodAngle.put(2.7, 30.0);

        // distanceToRPM.put(3.0, 2000.0);
        // distanceToHoodAngle.put(3.0, 30.0);

        // distanceToRPM.put(3.6, 2100.0);
        // distanceToHoodAngle.put(3.6, 30.0);

        // distanceToRPM.put(4.0, 2250.0);
        // distanceToHoodAngle.put(4.0, 30.0);

        // distanceToRPM.put(5.5, 2400.0);
        // distanceToHoodAngle.put(5.5, 40.0);

        distanceToRPM.put(7.5, 3100.0);
        distanceToHoodAngle.put(7.5, 40.0);
    }
}
