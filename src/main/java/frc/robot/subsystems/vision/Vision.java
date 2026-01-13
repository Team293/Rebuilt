package frc.robot.subsystems.vision;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SpikeSystem {

    private final VisionIO.VisionIOInputs io = new VisionIO.VisionIOInputs();
    private final VisionIOPhotonCamera photonCameras;
    private final CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        super("Vision");
        this.drive = drive;
        this.photonCameras = new VisionIOPhotonCamera();
    }

    @Override
    public void onPeriodic() {
        for (EstimatedRobotPose pose : io.estimatedRobotPoses) {
            drive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds
            );
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        return useAsyncDataRefresher(io, photonCameras);
    }
}
