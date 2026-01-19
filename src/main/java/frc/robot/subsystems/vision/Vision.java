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

        System.out.println("IO!!!!!!!!!!!!!!!!!!!!: " + io.estimatedRobotPoses + "\n\n\n\n");
    }

    @Override
    public void onPeriodic() {
        int index = 0;
        System.out.println("Camera sizes " + CameraManager.getCameras().size());

        System.out.println("Poses " + io.estimatedRobotPoses.size());
        for (EstimatedRobotPose pose : io.estimatedRobotPoses) {
            Logger.recordOutput("EstimatedPose/" + String.valueOf(index), pose.estimatedPose.toPose2d());
            drive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds
            );
            index++;
        }
    }

    @Override
    protected Runnable setupDataRefresher() {
        System.out.println("IO!!!!!!!!!!!!!!!!!!!!: " + io.estimatedRobotPoses + "\n\n\n\n");
        photonCameras = new VisionIOPhotonCamera();
        return useAsyncDataRefresher(photonCameras);
    }
}
