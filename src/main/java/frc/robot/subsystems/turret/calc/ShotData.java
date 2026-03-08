package frc.robot.subsystems.turret.calc;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    static {
        // TODO: get data and fill these in
        distanceToRPM.put(20.0, 3000.0);
        distanceToHoodAngle.put(20.0, 15.0);
    }
}
