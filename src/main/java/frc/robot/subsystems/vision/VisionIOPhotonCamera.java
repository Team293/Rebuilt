package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.photon.CameraManager;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class VisionIOPhotonCamera implements VisionIO {

    private final List<EstimatedRobotPose> estimatedRobotPoses;
    private final Supplier<Pose2d> odometryPoseSupplier;

    public VisionIOPhotonCamera(Supplier<Pose2d> odometryPoseSupplier) {
        this.estimatedRobotPoses = new ArrayList<>();
        this.odometryPoseSupplier = odometryPoseSupplier;
    }

    @Override
    public void refreshData() {
        Pose2d currentOdometryPose = odometryPoseSupplier.get();

        var newPoses = CameraManager.getCameras().stream()
                .map(camera -> {
                    // give the estimator the current odometry pose so single-tag fallback is accurate
                    camera.setReferencePose(currentOdometryPose);
                    return camera.getEstimatedRobotPose();
                })
                .filter(Objects::nonNull)
                .toList();
        estimatedRobotPoses.clear();
        estimatedRobotPoses.addAll(newPoses);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.estimatedRobotPoses = estimatedRobotPoses.stream()
                .map(pose -> pose.estimatedPose)
                .toArray(Pose3d[]::new);
    }

    @Override
    public List<EstimatedRobotPose> getEstimatedRobotPoses() {
        return new ArrayList<>(estimatedRobotPoses);
    }
}


