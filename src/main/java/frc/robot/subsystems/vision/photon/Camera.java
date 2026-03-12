package frc.robot.subsystems.vision.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class Camera {
    // ambiguity refers to pose confidence. lower ambiguity means higher confidence, and vice versa. 0.2 is recommended
    private static final double AMBIGUITY_THRESHOLD = 0.2;

    private transient final PhotonCamera photonCamera;
    private final String id;
    private transient final PhotonPoseEstimator poseEstimator;
    private final Transform3d robotToCamera;

    public Camera(String id, Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        this.id = id;
        this.photonCamera = new PhotonCamera(id);
        this.poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera
        );
        // fall back to lowest-ambiguity single-tag strategy when multi-tag isn't available
        this.poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Get the best estimated robot pose from the camera's unread results (using the most recent timestamp)
     * @return The best EstimatedRobotPose, or null if none are available
     */
    public EstimatedRobotPose getEstimatedRobotPose() {
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        // get the best estimated pose from the results (using the most recent timestamp)
        Optional<EstimatedRobotPose> bestPose = Optional.empty();
        // smallest possible timestamp, so any real timestamp will be greater than this
        double bestTimestamp = Double.NEGATIVE_INFINITY;

        for (PhotonPipelineResult result : results) {
            // check if the pose is within ambiguity threshold
            if (result.hasTargets() && isAmbiguous(result)) {
                continue;
            }

            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

            // if the pose is present and the timestamp is greater than the best timestamp, update the best pose and timestamp
            if (estimatedPose.isPresent()) {
                double timestamp = result.getTimestampSeconds();
                if (timestamp > bestTimestamp) {
                    bestTimestamp = timestamp;
                    bestPose = estimatedPose;
                }
            }
        }

        return bestPose.orElse(null);
    }

    private static boolean isAmbiguous(PhotonPipelineResult result) {
        var bestTarget = result.getBestTarget();
        if (bestTarget == null) return false;

        double ambiguity = bestTarget.getPoseAmbiguity();
        return ambiguity > AMBIGUITY_THRESHOLD;
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public String getId() {
        return id;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    /**
     * Sets the reference pose used by the pose estimator for single-tag fallback estimation.
     * Should be called with the current odometry pose before each call to getEstimatedRobotPose().
     * @param referencePose the current best-estimate robot pose from odometry
     */
    public void setReferencePose(edu.wpi.first.math.geometry.Pose2d referencePose) {
        poseEstimator.setReferencePose(referencePose);
    }
}
