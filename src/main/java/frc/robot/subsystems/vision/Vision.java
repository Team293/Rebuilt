package frc.robot.subsystems.vision;

import frc.lib.SubsystemDataProcessor;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class Vision extends SpikeSystem<VisionIOInputs> {
    private static final double AMBIGUITY_THRESHOLD = 0.2;
    private static final double HEADING_MAX_AGE_SECONDS = 0.5;

    private Rotation2d aprilTagHeading;
    private double aprilTagHeadingTimestamp = Double.NEGATIVE_INFINITY;

    private VisionIO visionIO;
    private final CommandSwerveDrivetrain drive;

    public Vision(CommandSwerveDrivetrain drive) {
        super("Vision", new VisionIOInputsAutoLogged());

        this.drive = drive;
    }

    @Override
    public void onPeriodic() {
        // Legacy logs contain poses without the timestamps and quality needed for fusion.
        int count = io.estimatedRobotPoses.length;
        if (io.timestampsSeconds.length != count || io.tagCounts.length != count
                || io.averageTagDistancesMeters.length != count || io.maxTagAmbiguities.length != count) return;

        // update drive with vision measurements
        for (int i = 0; i < count; i++) {
            int numTags = io.tagCounts[i];
            if (numTags == 0) continue;
            if (io.maxTagAmbiguities[i] > AMBIGUITY_THRESHOLD) continue;
            double avgDist = io.averageTagDistancesMeters[i];

            // Optional hard rejection (VERY useful in matches)
            if (avgDist > 5.0) continue;

            if (io.timestampsSeconds[i] > aprilTagHeadingTimestamp) {
                aprilTagHeading = io.estimatedRobotPoses[i].toPose2d().getRotation();
                aprilTagHeadingTimestamp = io.timestampsSeconds[i];
            }

            double scale = 1 + (avgDist * avgDist / 50.0);

            // Trust multi-tag solutions more
            if (numTags >= 2) {
                scale *= 0.7;
            } else {
                scale *= 1.3;
            }

            var stdDevs = CommandSwerveDrivetrain.kDefaultVisionStdDevs.times(scale);

            drive.addVisionMeasurement(
                io.estimatedRobotPoses[i].toPose2d(),
                io.timestampsSeconds[i],
                stdDevs
            );
        }    
    }

    @Override
    protected Runnable setupDataRefresher() {
        if (RobotBase.isSimulation()) {
            this.visionIO = new VisionIO() {
                @Override
                public void updateInputs(VisionIOInputs inputs) {}

                @Override
                public void refreshData() {}
            };
            return useDataRefresher(visionIO);
        }
        this.visionIO = new VisionIOPhotonCamera(() -> RobotContainer.getDrive().getPose());
        // Vision publishes immutable batches itself; never wait for camera processing
        // under the generic hardware refresh lock on the main thread.
        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {}, visionIO);
        return useDataRefresher(visionIO);
    }

    /** Returns the latest accepted AprilTag heading while its measurement is fresh. */
    public Optional<Rotation2d> getAprilTagHeading() {
        double age = Timer.getFPGATimestamp() - aprilTagHeadingTimestamp;
        return age >= 0.0 && age <= HEADING_MAX_AGE_SECONDS
                ? Optional.ofNullable(aprilTagHeading)
                : Optional.empty();
    }

    public Pose2d getEstimatedPositionFromCameras() {
        return io.estimatedRobotPoses.length == 0 ? null : io.estimatedRobotPoses[0].toPose2d();
    }
}
