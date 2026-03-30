package frc.robot.subsystems.targeting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToTOFConstant = new InterpolatingDoubleTreeMap();

    // 2m: 1.06s
    // 3.16m: 1.1s

    static {
        // distanceToRPM.put(3.14, 2300.0);
        // distanceToHoodAngle.put(3.14, 30.0);

        // distanceToRPM.put(4.7, 2550.0);
        // distanceToHoodAngle.put(4.7, 36.0);

        // distanceToRPM.put(1.5, 1900.0);
        // distanceToHoodAngle.put(1.5, 15.0);

        // distanceToRPM.put(2.5, 2300.0);
        // distanceToHoodAngle.put(2.5, 25.0);

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

        // distanceToRPM.put(7.5, 3100.0);
        // distanceToHoodAngle.put(7.5, 40.0);


        distanceToRPM.put(1.0, 1800.0);
        distanceToHoodAngle.put(1.0, 15.0);
        distanceToTOFConstant.put(1.0, 0.8);
        distanceToTOFConstant.put(4.0, 0.9  );

        distanceToRPM.put(2.0, 2050.0);
        distanceToHoodAngle.put(2.0, 23.0);

        distanceToRPM.put(3.0, 2200.0);
        distanceToHoodAngle.put(3.0, 30.0);

        distanceToRPM.put(4.0, 2500.0);
        distanceToHoodAngle.put(4.0, 35.0);

        distanceToRPM.put(5.3, 2750.0);
        distanceToHoodAngle.put(5.3, 35.0);

        distanceToRPM.put(17.069, 5000.0);
        distanceToHoodAngle.put(17.069, 45.0);
    }
}
