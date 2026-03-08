package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

public interface VisionIO extends BaseIO<VisionIO.VisionIOInputs>, IORefresher {

    @AutoLog
    public static class VisionIOInputs extends BaseInputClass {
        public Pose3d[] estimatedRobotPoses = new Pose3d[0];
    }

    List<EstimatedRobotPose> getEstimatedRobotPoses();
}
