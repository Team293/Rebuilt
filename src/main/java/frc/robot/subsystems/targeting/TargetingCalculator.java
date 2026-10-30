package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.turret.Turret;

/** Publishes one shot per update; all reads return the same immutable result. */
class TargetingCalculator {
    private TargetingSolution solution = new TargetingSolution(
            new Translation2d(), 0, 0, new Translation2d(), new Translation2d(),
            new ShotCompensation.AdjustedShot(0, 0, 0, 0, 0));

    void update(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation2d target) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds, robotPose.getRotation());
        var solved = MovingShotSolver.solve(
                robotPose, fieldSpeeds, target, Turret.TURRET_OFFSET_FROM_CENTER);
        // The solver is stateless: target and odometry changes cannot blend old samples.
        // Filtering its final vector would delay compensation and alter launch geometry.
        solution = new TargetingSolution(solved.toGoal(),
                target.getDistance(solved.pivotPosition()), solved.timeOfFlightS(),
                solved.pivotPosition(), solved.pivotVelocity(), solved.shot());
    }

    TargetingSolution getSolution() {
        return solution;
    }
}
