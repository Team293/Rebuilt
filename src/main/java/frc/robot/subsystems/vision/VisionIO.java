package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO extends BaseIO<VisionIO.VisionIOInputs>, IORefresher {

    @AutoLog
    public static class VisionIOInputs extends BaseInputClass {
        // Aligned by observation index; populated together once per main-thread loop.
        public Pose3d[] estimatedRobotPoses = new Pose3d[0];
        public double[] timestampsSeconds = new double[0];
        public int[] tagCounts = new int[0];
        public double[] averageTagDistancesMeters = new double[0];
        public double[] maxTagAmbiguities = new double[0];
    }
}
