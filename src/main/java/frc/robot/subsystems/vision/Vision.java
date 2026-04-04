package frc.robot.subsystems.vision;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SpikeSystem<VisionIOInputs> {
    // ==================== VISION FILTERING CONSTANTS ====================
    
    /** Maximum pose ambiguity allowed for a target to be used (0.0 = perfect, 1.0 = bad) */
    private static final double AMBIGUITY_THRESHOLD = 0.2;
    
    /** Maximum angular velocity (deg/sec) at which vision measurements are accepted.
     *  Vision is unreliable during fast rotations due to motion blur. */
    private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 270.0;
    
    /** Maximum distance (meters) a vision pose can differ from odometry to be accepted.
     *  Prevents erroneous measurements from "teleporting" the robot. */
    private static final double MAX_POSE_DEVIATION_METERS = 1.0;
    
    /** Maximum heading difference (degrees) a vision pose can differ from odometry. */
    private static final double MAX_HEADING_DEVIATION_DEGREES = 30.0;
    
    /** Minimum distance (meters) to a tag for reliable pose estimation.
     *  Tags closer than this suffer from lens distortion and poor localization. */
    private static final double MIN_TAG_DISTANCE_METERS = 0.3;
    
    /** Minimum number of tags required for highest confidence measurements */
    private static final int MULTI_TAG_THRESHOLD = 2;
    
    /** Weight multiplier for multi-tag measurements (more tags = more trust) */
    private static final double MULTI_TAG_WEIGHT_BONUS = 2.0;

    // ==================== INSTANCE VARIABLES ====================
    
    private VisionIO visionIO;
    private final CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        super("Vision", new VisionIOInputsAutoLogged());
        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        // Get current robot state for filtering
        double robotOmegaDegPerSec = Math.abs(drive.getRobotOmegaDegPerSec());
        Pose2d currentOdometryPose = drive.getPose();
        
        // Log rejection reasons for debugging
        boolean rejectedDueToRotation = false;
        int rejectedDueToOutlier = 0;
        int rejectedDueToAmbiguity = 0;
        int rejectedDueToTooClose = 0;
        int acceptedMeasurements = 0;
        
        // === FILTER 1: Angular velocity rejection ===
        // Skip ALL vision processing if robot is rotating too fast
        if (robotOmegaDegPerSec > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            rejectedDueToRotation = true;
            logFilteringStats(rejectedDueToRotation, rejectedDueToOutlier, rejectedDueToAmbiguity, rejectedDueToTooClose, acceptedMeasurements);
            return;
        }
        
        // Collect valid poses for fusion
        List<ValidatedPose> validPoses = new ArrayList<>();
        
        for (EstimatedRobotPose pose : visionIO.getEstimatedRobotPoses()) {
            if (pose == null) {
                continue;
            }
            
            // === FILTER 2: Ambiguity check ===
            double avgDist = 0;
            boolean passesAmbiguity = true;
            boolean passesMinDistance = true;
            int numTags = pose.targetsUsed.size();
            
            if (!pose.targetsUsed.isEmpty()) {
                double totalDist = 0;
                for (var target : pose.targetsUsed) {
                    if (target.poseAmbiguity > AMBIGUITY_THRESHOLD) {
                        passesAmbiguity = false;
                        break;
                    }
                    double tagDist = target.getBestCameraToTarget().getTranslation().getNorm();
                    totalDist += tagDist;
                    
                    // === FILTER 2.5: Minimum tag distance check ===
                    // Tags too close to the camera have poor pose estimation due to lens distortion
                    if (tagDist < MIN_TAG_DISTANCE_METERS) {
                        passesMinDistance = false;
                        break;
                    }
                }
                avgDist = totalDist / numTags;
            }
            
            if (!passesMinDistance) {
                rejectedDueToTooClose++;
                continue;
            }
            
            if (!passesAmbiguity) {
                rejectedDueToAmbiguity++;
                continue;
            }
            
            // === FILTER 3: Outlier rejection ===
            Pose2d visionPose2d = pose.estimatedPose.toPose2d();
            
            double positionDeviation = currentOdometryPose.getTranslation()
                    .getDistance(visionPose2d.getTranslation());
            
            double headingDeviation = Math.abs(
                    currentOdometryPose.getRotation().minus(visionPose2d.getRotation()).getDegrees()
            );
            
            // Normalize heading deviation to [-180, 180]
            if (headingDeviation > 180) {
                headingDeviation = 360 - headingDeviation;
            }
            
            if (positionDeviation > MAX_POSE_DEVIATION_METERS || 
                headingDeviation > MAX_HEADING_DEVIATION_DEGREES) {
                rejectedDueToOutlier++;
                Logger.recordOutput("Vision/RejectedPose", visionPose2d);
                Logger.recordOutput("Vision/RejectedPositionDeviation", positionDeviation);
                Logger.recordOutput("Vision/RejectedHeadingDeviation", headingDeviation);
                continue;
            }
            
            // Pose passed all filters - add to valid poses for fusion
            validPoses.add(new ValidatedPose(
                    visionPose2d,
                    pose.timestampSeconds,
                    avgDist,
                    numTags
            ));
        }
        
        // === FUSION: Combine multiple camera poses into single measurement ===
        if (!validPoses.isEmpty()) {
            FusedMeasurement fused = fuseValidPoses(validPoses);
            
            // Calculate standard deviations based on fused confidence
            double stdDevMultiplier = 1 + ((fused.avgDistance * fused.avgDistance) / 30);
            
            // Reduce std devs if we have multi-tag measurements
            if (fused.avgTagCount >= MULTI_TAG_THRESHOLD) {
                stdDevMultiplier *= 0.7; // More confident with multi-tag
            }
            
            drive.addVisionMeasurement(
                    fused.pose,
                    fused.timestamp,
                    CommandSwerveDrivetrain.kDefaultVisionStdDevs.times(stdDevMultiplier)
            );
            
            acceptedMeasurements = validPoses.size();
            Logger.recordOutput("Vision/FusedPose", fused.pose);
            Logger.recordOutput("Vision/FusedStdDevMultiplier", stdDevMultiplier);
            Logger.recordOutput("Vision/FusedAvgTagCount", fused.avgTagCount);
        }
        
        logFilteringStats(rejectedDueToRotation, rejectedDueToOutlier, rejectedDueToAmbiguity, rejectedDueToTooClose, acceptedMeasurements);
    }
    
    /**
     * Fuses multiple validated camera poses into a single weighted measurement.
     * Weights are based on number of tags (more tags = more weight) and distance (closer = more weight).
     */
    private FusedMeasurement fuseValidPoses(List<ValidatedPose> poses) {
        if (poses.size() == 1) {
            // Single pose - no fusion needed
            ValidatedPose p = poses.get(0);
            return new FusedMeasurement(p.pose, p.timestamp, p.avgDistance, p.numTags);
        }
        
        // Calculate weights for each pose
        double totalWeight = 0;
        double[] weights = new double[poses.size()];
        
        for (int i = 0; i < poses.size(); i++) {
            ValidatedPose p = poses.get(i);
            
            // Weight increases with more tags, decreases with distance
            double tagWeight = p.numTags >= MULTI_TAG_THRESHOLD ? MULTI_TAG_WEIGHT_BONUS : 1.0;
            double distWeight = 1.0 / (1.0 + p.avgDistance); // Inverse distance weighting
            
            weights[i] = tagWeight * distWeight;
            totalWeight += weights[i];
        }
        
        // Weighted average of positions
        double weightedX = 0;
        double weightedY = 0;
        double weightedSin = 0;
        double weightedCos = 0;
        double weightedTimestamp = 0;
        double weightedDistance = 0;
        double weightedTagCount = 0;
        
        for (int i = 0; i < poses.size(); i++) {
            ValidatedPose p = poses.get(i);
            double w = weights[i] / totalWeight; // Normalize weight
            
            weightedX += p.pose.getX() * w;
            weightedY += p.pose.getY() * w;
            
            // Use sin/cos for proper angle averaging
            weightedSin += Math.sin(p.pose.getRotation().getRadians()) * w;
            weightedCos += Math.cos(p.pose.getRotation().getRadians()) * w;
            
            weightedTimestamp += p.timestamp * w;
            weightedDistance += p.avgDistance * w;
            weightedTagCount += p.numTags * w;
        }
        
        Pose2d fusedPose = new Pose2d(
                new Translation2d(weightedX, weightedY),
                new Rotation2d(weightedCos, weightedSin)
        );
        
        return new FusedMeasurement(fusedPose, weightedTimestamp, weightedDistance, weightedTagCount);
    }
    
    private void logFilteringStats(boolean rejectedRotation, int rejectedOutlier, 
                                   int rejectedAmbiguity, int rejectedTooClose, int accepted) {
        Logger.recordOutput("Vision/RejectedDueToRotation", rejectedRotation);
        Logger.recordOutput("Vision/RejectedDueToOutlier", rejectedOutlier);
        Logger.recordOutput("Vision/RejectedDueToAmbiguity", rejectedAmbiguity);
        Logger.recordOutput("Vision/RejectedDueToTooClose", rejectedTooClose);
        Logger.recordOutput("Vision/AcceptedMeasurements", accepted);
    }

    @Override
    protected Runnable setupDataRefresher() {
        this.visionIO = new VisionIOPhotonCamera(() -> RobotContainer.getDrive().getPose());
        return useAsyncDataRefresher(visionIO);
    }

    public Pose2d getEstimatedPositionFromCameras() {
        if (visionIO.getEstimatedRobotPoses().isEmpty()) {
            return null;
        }

        for (EstimatedRobotPose pose : visionIO.getEstimatedRobotPoses()) {
            if (pose != null) {
                return pose.estimatedPose.toPose2d();
            }
        }
        return null;
    }
    
    // ==================== INNER CLASSES ====================
    
    /** Holds a validated pose with its metadata for fusion */
    private static class ValidatedPose {
        final Pose2d pose;
        final double timestamp;
        final double avgDistance;
        final int numTags;
        
        ValidatedPose(Pose2d pose, double timestamp, double avgDistance, int numTags) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.avgDistance = avgDistance;
            this.numTags = numTags;
        }
    }
    
    /** Result of fusing multiple poses */
    private static class FusedMeasurement {
        final Pose2d pose;
        final double timestamp;
        final double avgDistance;
        final double avgTagCount;
        
        FusedMeasurement(Pose2d pose, double timestamp, double avgDistance, double avgTagCount) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.avgDistance = avgDistance;
            this.avgTagCount = avgTagCount;
        }
    }
}
