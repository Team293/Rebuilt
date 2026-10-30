package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.photon.CameraManager;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

public class VisionIOPhotonCamera implements VisionIO {

    // Only immutable values cross the worker/main-thread boundary, never Photon targets.
    private record Observation(Pose3d pose, double timestampSeconds, int tagCount,
            double averageTagDistanceMeters, double maxTagAmbiguity) {
        static Observation from(EstimatedRobotPose estimate) {
            double totalDistance = 0.0;
            double maxAmbiguity = 0.0;
            for (var target : estimate.targetsUsed) {
                totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                maxAmbiguity = Math.max(maxAmbiguity, target.getPoseAmbiguity());
            }
            int tagCount = estimate.targetsUsed.size();
            return new Observation(estimate.estimatedPose, estimate.timestampSeconds, tagCount,
                    tagCount == 0 ? 0.0 : totalDistance / tagCount, maxAmbiguity);
        }
    }

    private final AtomicReference<List<Observation>> pendingObservations = new AtomicReference<>(List.of());
    private final Supplier<Pose2d> odometryPoseSupplier;
    private final Function<Pose2d, List<EstimatedRobotPose>> cameraPoseSupplier;

    public VisionIOPhotonCamera(Supplier<Pose2d> odometryPoseSupplier) {
        this(odometryPoseSupplier, currentOdometryPose -> CameraManager.getCameras().stream()
                .map(camera -> {
                    // give the estimator the current odometry pose so single-tag fallback is accurate
                    camera.setReferencePose(currentOdometryPose);
                    return camera.getEstimatedRobotPose();
                })
                .filter(Objects::nonNull)
                .toList());
    }

    VisionIOPhotonCamera(Supplier<Pose2d> odometryPoseSupplier,
            Function<Pose2d, List<EstimatedRobotPose>> cameraPoseSupplier) {
        this.odometryPoseSupplier = odometryPoseSupplier;
        this.cameraPoseSupplier = cameraPoseSupplier;
    }

    @Override
    public void refreshData() {
        // Camera polling, estimation and target processing all stay on the worker.
        var batch = cameraPoseSupplier.apply(odometryPoseSupplier.get()).stream()
                .map(Observation::from)
                .toList();
        if (batch.isEmpty()) return;

        pendingObservations.getAndUpdate(pending -> {
            var combined = new ArrayList<>(pending);
            combined.addAll(batch);
            return List.copyOf(combined);
        });
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Atomically claim every complete batch since the last loop, exactly once.
        var observations = pendingObservations.getAndSet(List.of());
        inputs.estimatedRobotPoses = observations.stream().map(Observation::pose).toArray(Pose3d[]::new);
        inputs.timestampsSeconds = observations.stream().mapToDouble(Observation::timestampSeconds).toArray();
        inputs.tagCounts = observations.stream().mapToInt(Observation::tagCount).toArray();
        inputs.averageTagDistancesMeters = observations.stream()
                .mapToDouble(Observation::averageTagDistanceMeters).toArray();
        inputs.maxTagAmbiguities = observations.stream().mapToDouble(Observation::maxTagAmbiguity).toArray();
    }
}


