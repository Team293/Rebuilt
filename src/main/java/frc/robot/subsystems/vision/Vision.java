package frc.robot.subsystems.vision;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import org.littletonrobotics.junction.Logger;
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
            if (pose == null) {
                continue;
            }
            double avgDist = 0;
            boolean usePose = true;

            // calculate the average distance to the targets
            // used to calculate the standard deviations of the vision measurement
            if (!pose.targetsUsed.isEmpty()) {
                double totalDist = 0;
                for (var target : pose.targetsUsed) {
                    if (target.poseAmbiguity > AMBIGUITY_THRESHOLD) {
                        usePose = false;
                        break;
                    }

                    totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
                }

                avgDist = totalDist / pose.targetsUsed.size();
            }

            if (!usePose) {
                continue;
            }

            drive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds,
                    CommandSwerveDrivetrain.kDefaultVisionStdDevs.times(1 + ((avgDist * avgDist) / 30))
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
