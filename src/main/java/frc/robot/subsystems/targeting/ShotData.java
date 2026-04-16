package frc.robot.subsystems.targeting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToTOFConstant = new InterpolatingDoubleTreeMap(); //TOF = Time of Flight
    
    static {
        distanceToTOFConstant.put(1.0, 0.8);
        distanceToTOFConstant.put(3.0, 1.2);
        distanceToTOFConstant.put(4.0, 1.4);
        distanceToTOFConstant.put(5.0, 1.6);
        distanceToTOFConstant.put(6.0, 1.9);
        
        distanceToTOFConstant.put(10.0, 4.0);

        distanceToRPM.put(2.0, 2000.0);
        distanceToHoodAngle.put(2.0, 25.0);

        distanceToRPM.put(2.5, 2150.0);
        distanceToHoodAngle.put(2.5, 25.0);

        distanceToRPM.put(3.0, 2200.0);
        distanceToHoodAngle.put(3.0, 25.0);

        distanceToRPM.put(3.5, 2400.0);
        distanceToHoodAngle.put(3.5, 27.0);

        distanceToRPM.put(4.0, 2500.0);
        distanceToHoodAngle.put(4.0, 27.0);

        distanceToRPM.put(4.5, 2600.0);
        distanceToHoodAngle.put(4.5, 27.0);

        distanceToRPM.put(4.9, 2700.0);
        distanceToHoodAngle.put(4.9, 29.0);

        distanceToRPM.put(5.88, 3000.0);
        distanceToHoodAngle.put(5.88, 35.0);

        distanceToRPM.put(17.069, 5000.0);
        distanceToHoodAngle.put(17.069, 45.0);
    }
}
