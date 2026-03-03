package frc.robot.subsystems.turret.calc;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotData {
    public static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    static {
        // TODO: get data and fill these in
        distanceToRPM.put(0.0, 0.0);
        distanceToHoodAngle.put(0.0, 0.0);
    }

}
