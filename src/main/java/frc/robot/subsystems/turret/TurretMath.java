package frc.robot.subsystems.turret;

public class TurretMath {

    private static double offsetDegrees = 0.0;

    private static double mod(double x, double m) {
        return ((x % m) + m) % m;
    }

    /**
     * Computes absolute turret position using CRT.
     *
     * @param enc13 Encoder on 13:140 ratio, in [0,1)
     * @param enc11 Encoder on 11:140 ratio, in [0,1)
     * @return turret position in revolutions (mod 143)
     */
    public static double getTurretPosition(double enc13, double enc11) {
        double offsetRevs = offsetDegrees / 360.0;

        enc13 = mod(enc13 - offsetRevs, 1.0);
        enc11 = mod(enc11 - offsetRevs, 1.0);

        double diff = enc11 - enc13;

        double k = mod(6.0 * diff, 11.0);

        return enc13 + 13.0 * k;
    }

    public static double positionToDegrees(double positionRevs) {
        return mod(positionRevs, 1.0) * 360.0;
    }

    public static double positionToRadians(double positionRevs) {
        return mod(positionRevs, 1.0) * 2.0 * Math.PI;
    }
}
