package frc.robot.subsystems.vision.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class Camera {
    private transient final PhotonCamera photonCamera;
    private final String id;
    private transient final PhotonPoseEstimator poseEstimator;
    private final Transform3d cameraToRobot;

    public Camera(String id, Transform3d cameraToRobot) {
        this.cameraToRobot = cameraToRobot;
        this.id = id;
        this.photonCamera = new PhotonCamera(id);
        this.poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraToRobot
        );
    }

    /**
     * Get the best estimated robot pose from the camera's unread results (using the most recent timestamp)
     * @return The best EstimatedRobotPose, or null if none are available
     */
    public EstimatedRobotPose getEstimatedRobotPose() {
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        // PhotonPipelineResult result = photonCamera.getLatestResult();
        // System.out.println("Pose from camera " + this.id + " " + result);
        
        Optional<EstimatedRobotPose> bestPose = Optional.empty();
        double bestTimestamp = Double.NEGATIVE_INFINITY;

        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
            if (estimatedPose.isPresent()) {
                double timestamp = result.getTimestampSeconds();
                if (timestamp > bestTimestamp) {
                    bestTimestamp = timestamp;
                    bestPose = estimatedPose;
                }
            }
        }

        return bestPose.orElse(null);

        // var estimatedPose = poseEstimator.update(result).orElse(null);
        
        // Logger.recordOutput("EstimatedPose/" + this.id, estimatedPose.estimatedPose);  

        // return estimatedPose;
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

    public Transform3d getCameraToRobot() {
        return cameraToRobot;
    }
}
