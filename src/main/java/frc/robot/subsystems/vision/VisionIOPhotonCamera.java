package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.subsystem.IORefresher;
import frc.robot.subsystems.vision.photon.Camera;
import frc.robot.subsystems.vision.photon.CameraManager;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class VisionIOPhotonCamera implements VisionIO, IORefresher {

    private final List<EstimatedRobotPose> estimatedRobotPoses;

    public VisionIOPhotonCamera() {
        this.estimatedRobotPoses = new ArrayList<>();
    }

    @Override
    public void refreshData() {
        var newPoses = CameraManager.getCameras().stream()
                .map(Camera::getEstimatedRobotPose)
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
