package frc.robot.subsystems.vision;

import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

public interface VisionIO extends BaseIO<VisionIO.VisionIOInputs> {

    @AutoLog
    public static class VisionIOInputs extends BaseInputClass {
        public List<EstimatedRobotPose> estimatedRobotPoses = List.of();
    }
}
