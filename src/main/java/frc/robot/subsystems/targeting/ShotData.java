package frc.robot.subsystems.targeting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToTOFConstant = new InterpolatingDoubleTreeMap(); //TOF = Time of Flight
    
    static {
        distanceToRPM.put(1.0, 1800.0);
        distanceToHoodAngle.put(1.0, 15.0);
        distanceToTOFConstant.put(1.0, 0.8);
        distanceToTOFConstant.put(4.0, 0.9  );

        distanceToRPM.put(2.0, 2050.0);
        distanceToHoodAngle.put(2.0, 23.0);

        distanceToRPM.put(3.0, 2200.0);
        distanceToHoodAngle.put(3.0, 30.0);

        distanceToRPM.put(4.0, 2500.0);
        distanceToHoodAngle.put(4.0, 32.0);

        distanceToRPM.put(5.3, 2650.0);
        distanceToHoodAngle.put(5.3, 35.0);

        distanceToRPM.put(17.069, 5000.0);
        distanceToHoodAngle.put(17.069, 45.0);
    }
}
