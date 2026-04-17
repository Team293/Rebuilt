package frc.robot.subsystems.vision;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class Vision extends SpikeSystem<VisionIOInputs> {
    private static final double AMBIGUITY_THRESHOLD = 0.2;

    private VisionIO visionIO;
    private final CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        super("Vision", new VisionIOInputsAutoLogged());

        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        // update drive with vision measurements
        for (EstimatedRobotPose pose : visionIO.getEstimatedRobotPoses()) {
            if (pose == null) continue;

            double avgDist = 0.0;
            boolean usePose = true;
            int numTags = pose.targetsUsed.size();

            if (numTags == 0) continue;

            double totalDist = 0.0;

            for (var target : pose.targetsUsed) {
                // Reject high ambiguity targets
                if (target.poseAmbiguity > AMBIGUITY_THRESHOLD) {
                    usePose = false;
                    break;
                }

                totalDist += target.getBestCameraToTarget()
                                .getTranslation()
                                .getNorm();
            }

            if (!usePose) continue;

            avgDist = totalDist / numTags;

            // Optional hard rejection (VERY useful in matches)
            if (avgDist > 5.0) continue;

            double scale = 1 + (avgDist * avgDist / 50.0);

            // Trust multi-tag solutions more
            if (numTags >= 2) {
                scale *= 0.7;
            } else {
                scale *= 1.3;
            }

            var stdDevs = CommandSwerveDrivetrain.kDefaultVisionStdDevs.times(scale);

            drive.addVisionMeasurement(
                pose.estimatedPose.toPose2d(),
                pose.timestampSeconds,
                stdDevs
            );
        }    
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
}
