package frc.robot.subsystems.vision;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.photon.CameraManager;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SpikeSystem<VisionIOInputs> {

    private VisionIOPhotonCamera photonCameras;
    private final CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        super("Vision", new VisionIO.VisionIOInputs());

        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        int index = 0;
        // update drive with vision measurements
        for (EstimatedRobotPose pose : io.estimatedRobotPoses) {
            Logger.recordOutput("EstimatedPose/" + index, pose.estimatedPose.toPose2d());
            drive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds
            );
            index++;
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        photonCameras = new VisionIOPhotonCamera();
        return useAsyncDataRefresher(photonCameras);
    }
}
