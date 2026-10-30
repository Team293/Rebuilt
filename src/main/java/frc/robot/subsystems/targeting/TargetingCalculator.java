package frc.robot.subsystems.targeting;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.turret.Turret;

/** Owns filter history; only update advances it, while reads have no side effects. */
class TargetingCalculator {
    private final MedianFilter distMedian = new MedianFilter(5);
    private final LinearFilter distIIR = LinearFilter.singlePoleIIR(0.06, 0.02);
    private final LinearFilter toGoalXFilter = LinearFilter.singlePoleIIR(0.04, 0.02);
    private final LinearFilter toGoalYFilter = LinearFilter.singlePoleIIR(0.04, 0.02);
    private Translation2d previousTarget;
    private long previousResetRevision;
    private TargetingSolution solution = new TargetingSolution(
            new Translation2d(), 0, 0, new Pose2d(), new Pose2d());

    void update(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds,
            Translation2d target, long resetRevision) {
        boolean reseed = !target.equals(previousTarget) || resetRevision != previousResetRevision;
        Translation2d turretPivotNow = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
        double rawDistance = target.getDistance(turretPivotNow);
        if (reseed) {
            distMedian.reset();
            seed(distIIR, rawDistance);
        }
        double staticDistance = distIIR.calculate(distMedian.calculate(rawDistance));
        double tof = ShotData.distanceToTOFConstant.get(staticDistance);
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds, robotPose.getRotation());
        Translation2d predictedRobotPos = robotPose.getTranslation().plus(new Translation2d(
                speeds.vxMetersPerSecond * tof, speeds.vyMetersPerSecond * tof));
        Rotation2d predictedHeading = robotPose.getRotation().plus(
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * tof));
        Translation2d predictedTurretPivot = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(predictedHeading).plus(predictedRobotPos);
        Translation2d toGoalComp = target.minus(predictedTurretPivot);
        if (reseed) {
            seed(toGoalXFilter, toGoalComp.getX());
            seed(toGoalYFilter, toGoalComp.getY());
        }
        Translation2d filteredToGoal = new Translation2d(
                toGoalXFilter.calculate(toGoalComp.getX()),
                toGoalYFilter.calculate(toGoalComp.getY()));
        solution = new TargetingSolution(filteredToGoal, staticDistance, tof,
                new Pose2d(predictedRobotPos, predictedHeading),
                new Pose2d(predictedTurretPivot, predictedHeading));
        previousTarget = target;
        previousResetRevision = resetRevision;
    }

    TargetingSolution getSolution() {
        return solution;
    }

    private static void seed(LinearFilter filter, double value) {
        filter.reset(new double[] {value}, new double[] {value});
    }
}
