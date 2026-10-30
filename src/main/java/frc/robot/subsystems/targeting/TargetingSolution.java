package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Immutable result shared by all targeting consumers for one robot loop. */
public record TargetingSolution(
        Translation2d toGoal,
        double staticDistanceMeters,
        double timeOfFlightSeconds,
        Pose2d predictedRobotPose,
        Pose2d predictedTurretPivot) {
    public double rangeMeters() {
        return toGoal.getNorm();
    }

    public Rotation2d fieldRelativeDirection() {
        return toGoal.getAngle();
    }
}
