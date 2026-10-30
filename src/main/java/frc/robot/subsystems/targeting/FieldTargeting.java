package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.FieldConstants;
import frc.robot.subsystems.targeting.Targeting.Target;

/** Field-position decisions in the blue-origin coordinate system used by odometry. */
public final class FieldTargeting {
    public static final double TARGET_HYSTERESIS_METERS = 0.20;
    // Conservative footprint at any heading, including bumpers (32.5-inch square).
    public static final double ROBOT_RADIUS_METERS = Units.inchesToMeters(32.5) / Math.sqrt(2);
    public static final double TRENCH_MARGIN_METERS = 0.15;
    // Tune against measured hood retraction time before driving at full speed.
    public static final double HOOD_RETRACTION_LOOKAHEAD_SECONDS = 0.50;

    private FieldTargeting() {}

    public static Target selectTarget(Pose2d pose, boolean redAlliance, Target previous) {
        double x = redAlliance ? FieldConstants.fieldLength - pose.getX() : pose.getX();
        double y = redAlliance ? FieldConstants.fieldWidth - pose.getY() : pose.getY();
        double boundary = FieldConstants.LinesVertical.allianceZone;
        if (previous == Target.HUB) boundary += TARGET_HYSTERESIS_METERS;
        else boundary -= TARGET_HYSTERESIS_METERS;
        if (x <= boundary) return Target.HUB;

        double center = FieldConstants.LinesHorizontal.center;
        if (previous == Target.SHUTTLE_LEFT && y >= center - TARGET_HYSTERESIS_METERS) {
            return Target.SHUTTLE_LEFT;
        }
        if (previous == Target.SHUTTLE_RIGHT && y <= center + TARGET_HYSTERESIS_METERS) {
            return Target.SHUTTLE_RIGHT;
        }
        return y > center ? Target.SHUTTLE_LEFT : Target.SHUTTLE_RIGHT;
    }

    /** Retract for the current footprint or its swept path during the hood's retraction time. */
    public static boolean requiresHoodRetraction(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds, pose.getRotation());
        Translation2d start = pose.getTranslation();
        Translation2d end = start.plus(new Translation2d(
                fieldSpeeds.vxMetersPerSecond * HOOD_RETRACTION_LOOKAHEAD_SECONDS,
                fieldSpeeds.vyMetersPerSecond * HOOD_RETRACTION_LOOKAHEAD_SECONDS));
        double padding = ROBOT_RADIUS_METERS + TRENCH_MARGIN_METERS;
        for (double x : new double[] {
                FieldConstants.LinesVertical.hubCenter, FieldConstants.LinesVertical.oppHubCenter}) {
            if (intersects(start, end, x - FieldConstants.RightTrench.depth / 2 - padding,
                    x + FieldConstants.RightTrench.depth / 2 + padding,
                    -padding, FieldConstants.RightTrench.width + padding)
                    || intersects(start, end, x - FieldConstants.LeftTrench.depth / 2 - padding,
                    x + FieldConstants.LeftTrench.depth / 2 + padding,
                    FieldConstants.fieldWidth - FieldConstants.LeftTrench.width - padding,
                    FieldConstants.fieldWidth + padding)) return true;
        }
        return false;
    }

    // Segment/AABB intersection: covers fast crossings even when both endpoints are outside.
    private static boolean intersects(Translation2d start, Translation2d end,
            double minX, double maxX, double minY, double maxY) {
        double enter = 0;
        double exit = 1;
        double[] origin = {start.getX(), start.getY()};
        double[] delta = {end.getX() - start.getX(), end.getY() - start.getY()};
        double[] min = {minX, minY};
        double[] max = {maxX, maxY};
        for (int axis = 0; axis < 2; axis++) {
            if (Math.abs(delta[axis]) < 1e-9) {
                if (origin[axis] < min[axis] || origin[axis] > max[axis]) return false;
            } else {
                double a = (min[axis] - origin[axis]) / delta[axis];
                double b = (max[axis] - origin[axis]) / delta[axis];
                enter = Math.max(enter, Math.min(a, b));
                exit = Math.min(exit, Math.max(a, b));
                if (enter > exit) return false;
            }
        }
        return true;
    }
}
