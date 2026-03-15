package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.subsystem.IORefresher;
import frc.robot.subsystems.vision.photon.Camera;
import frc.robot.subsystems.vision.photon.CameraManager;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class VisionIOPhotonCamera implements VisionIO, IORefresher {

    private final List<EstimatedRobotPose> estimatedRobotPoses;
    private final List<EstimatedRobotPose> estimatedRobotPosesView; // unmodifiable view
    private final Supplier<Pose2d> odometryPoseSupplier;

    // reused output buffer so we don't allocate a new Pose3d[] on every loop
    private Pose3d[] poseBuffer = new Pose3d[0];

    public VisionIOPhotonCamera(Supplier<Pose2d> odometryPoseSupplier) {
        this.estimatedRobotPoses = new ArrayList<>();
        this.estimatedRobotPosesView = Collections.unmodifiableList(estimatedRobotPoses);
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
        int size = estimatedRobotPoses.size();

        if (poseBuffer.length != size) {
            poseBuffer = new Pose3d[size];
        }
        for (int i = 0; i < size; i++) {
            poseBuffer[i] = estimatedRobotPoses.get(i).estimatedPose;
        }
        inputs.estimatedRobotPoses = poseBuffer;
    }

    @Override
    public List<EstimatedRobotPose> getEstimatedRobotPoses() {
        // return an unmodifiable view
        return estimatedRobotPosesView;
    }
}


