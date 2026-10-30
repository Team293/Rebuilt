package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import frc.robot.RobotContainer;

public class Targeting extends SubsystemBase {
    private static ShotCompensation.AdjustedShot shotData = new ShotCompensation.AdjustedShot(0.0, 0.0, 0.0, 0.0, 0.0);

    private Translation2d targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

    private static final double FIELD_WIDTH = FieldConstants.APRIL_TAG_LAYOUT.getFieldWidth(); // meters
    private static final double FIELD_LENGTH = FieldConstants.APRIL_TAG_LAYOUT.getFieldLength(); // meters

    private static final double shuttlingXOffset = 2.0;
    private static final double shuttlingYOffset = 2.5;

    private final TargetingCalculator calculator = new TargetingCalculator();

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
    
    public Targeting() {
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));
        Logger.recordOutput("Outreach/BasetballHoop", new Pose3d(FieldConstants.Outreach.basketballHoop, new Rotation3d()));

        SmartDashboard.putBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        SmartDashboard.putBoolean("OverrideRedAlliance", overrideRedAlliance);
    }

    public TargetingSolution getSolution() {
        return calculator.getSolution();
    }

    /**
     * Called once before CommandScheduler.run(), so every consumer sees the same solution.
     */
    public void update() {
        var drive = RobotContainer.getDrive();
        var state = drive.getStateCopy();
        overrideBlueAlliance = SmartDashboard.getBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        overrideRedAlliance = SmartDashboard.getBoolean("OverrideRedAlliance", overrideRedAlliance);
        currentTarget = requestedTarget == Target.AUTO
                ? FieldTargeting.selectTarget(state.Pose, isRedAlliance(), currentTarget)
                : requestedTarget;
        RobotContainer.getLEDController().switchPreset(currentTarget == Target.HUB ? "hub" : "shuttle");
        if (currentTarget == Target.HUB) {
            setPoseTargetingHub();
        } else if (currentTarget == Target.SHUTTLE_RIGHT) {
            setPoseTargetingShuttleRight();
        } else if (currentTarget == Target.SHUTTLE_LEFT) {
            setPoseTargetingShuttleLeft();
        }
        calculator.update(state.Pose, state.Speeds, targetPos, drive.getOdometryResetRevision());
        TargetingSolution solution = getSolution();
        Logger.recordOutput("Targeting/StaticDistance", solution.staticDistanceMeters());
        Logger.recordOutput("Targeting/PredictedRobotPos", solution.predictedRobotPose());
        Logger.recordOutput("Targeting/PredictedTurretPivot", solution.predictedTurretPivot());
        Logger.recordOutput("Targeting/ToGoalCompensation",
                targetPos.minus(solution.predictedTurretPivot().getTranslation()));
        Logger.recordOutput("Targeting/TimeOfFlight", solution.timeOfFlightSeconds());
        Logger.recordOutput("Targeting/DistanceToTarget", solution.rangeMeters());
        Logger.recordOutput("Targeting/AngleToTargetDeg", solution.fieldRelativeDirection().getDegrees());
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
     * Returns the current shot data
     * @return ShotCompensation.AdjustedShot shot data to use
     */
    public static ShotCompensation.AdjustedShot getShotData() {
        return shotData;
    }
}
