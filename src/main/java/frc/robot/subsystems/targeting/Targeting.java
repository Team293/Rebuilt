package frc.robot.subsystems.targeting;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import frc.robot.RobotContainer;

public class Targeting extends SubsystemBase {
    private final TargetingCalculator calculator = new TargetingCalculator();

    private Translation2d targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

    private static final double FIELD_WIDTH = FieldConstants.APRIL_TAG_LAYOUT.getFieldWidth(); // meters
    private static final double FIELD_LENGTH = FieldConstants.APRIL_TAG_LAYOUT.getFieldLength(); // meters

    private static final double shuttlingXOffset = 2.0;
    private static final double shuttlingYOffset = 2.5;

    public static enum Target {
        AUTO,
        HUB,
        SHUTTLE_RIGHT,
        SHUTTLE_LEFT
    }

    private Target currentTarget = Target.HUB;
    private Target requestedTarget = Target.AUTO;

    private boolean overrideRedAlliance = false;
    private boolean overrideBlueAlliance = false;
    private final Supplier<SwerveDriveState> driveStateSupplier;
    
    public Targeting() {
        this(() -> RobotContainer.getDrive().getStateCopy());
    }

    Targeting(Supplier<SwerveDriveState> driveStateSupplier) {
        this.driveStateSupplier = driveStateSupplier;
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));
        Logger.recordOutput("Outreach/BasetballHoop", new Pose3d(FieldConstants.Outreach.basketballHoop, new Rotation3d()));

        SmartDashboard.putBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        SmartDashboard.putBoolean("OverrideRedAlliance", overrideRedAlliance);
    }

    /** Returns the immutable solution published for this scheduler cycle. */
    public TargetingSolution getSolution() {
        return calculator.getSolution();
    }

    // Called once per periodic update using pose and velocity from the same drivetrain state.
    void updateShotSolution(Pose2d robotPose, ChassisSpeeds robotRelSpeeds) {
        calculator.update(robotPose, robotRelSpeeds, targetPos);
        var solution = getSolution();
        Logger.recordOutput("Targeting/StaticDistance", solution.staticDistanceMeters());
        Logger.recordOutput("Targeting/TurretPivot", solution.turretPivot());
        Logger.recordOutput("Targeting/TurretPivotVelocity", solution.turretPivotVelocity());
        Logger.recordOutput("Targeting/ToGoalCompensation", solution.toGoal());
        Logger.recordOutput("Targeting/TimeOfFlight", solution.timeOfFlightSeconds());
        Logger.recordOutput("Targeting/DistanceToTarget", solution.rangeMeters());
        Logger.recordOutput("Targeting/AngleToTargetDeg", solution.fieldRelativeDirection().getDegrees());
        Logger.recordOutput("Targeting/TurretFeedforwardDegPerSec", solution.shot().turretFF_deg_s());
    }

    /**
     * Periodically calculate the shot data given the target position and robot movement
     */
    @Override
    public void periodic() {
        var driveState = driveStateSupplier.get();
        overrideBlueAlliance = SmartDashboard.getBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        overrideRedAlliance = SmartDashboard.getBoolean("OverrideRedAlliance", overrideRedAlliance);
        currentTarget = requestedTarget == Target.AUTO
                ? FieldTargeting.selectTarget(driveState.Pose, isRedAlliance(), currentTarget)
                : requestedTarget;
        var ledController = RobotContainer.getLEDController();
        if (ledController != null) {
            ledController.switchPreset(currentTarget == Target.HUB ? "hub" : "shuttle");
        }
        if (currentTarget == Target.HUB) {
            setPoseTargetingHub();
        } else if (currentTarget == Target.SHUTTLE_RIGHT) {
            setPoseTargetingShuttleRight();
        } else if (currentTarget == Target.SHUTTLE_LEFT) {
            setPoseTargetingShuttleLeft();
        }
        updateShotSolution(driveState.Pose, driveState.Speeds);
        Logger.recordOutput("Targeting/RequestedTarget", requestedTarget.toString());
        Logger.recordOutput("Targeting/SelectedTarget", currentTarget.toString());
        Logger.recordOutput("Targeting/TargetPosition", targetPos);
    }

    /**
     * Set the target of the targeting subsystem. This will change the target position
     */
    public void setTarget(Target target) {
        requestedTarget = target;
    }

    private boolean isRedAlliance() {
        if (overrideRedAlliance) return true;
        if (overrideBlueAlliance) return false;
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    /**
     * Set the target location to center of the hub 
     */
    private void setPoseTargetingHub() {
        targetPos = isRedAlliance() ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
                : FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    }

    /**
     * Set the target location to 0, 0
     */
    private void setPoseTargetingShuttleRight() {
        if (isRedAlliance()) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, 0 + shuttlingYOffset);
        }
    }

    private void setPoseTargetingShuttleLeft() {
        if (isRedAlliance()) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, 0 + shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        }
    }
    
    /**
     * Returns this cycle's base shot data, before shooter distance trim or manual overrides.
     * @return ShotCompensation.AdjustedShot shot data to use
     */
    public ShotCompensation.AdjustedShot getShotData() {
        return getSolution().shot();
    }
}
