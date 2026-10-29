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
import frc.robot.subsystems.turret.Turret;

public class Targeting extends SubsystemBase {
    private static ShotCompensation.AdjustedShot shotData = new ShotCompensation.AdjustedShot(0.0, 0.0, 0.0, 0.0, 0.0);
    private static Translation2d compensatedToGoal = new Translation2d();

    private static Translation2d targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

    private static final double FIELD_WIDTH = FieldConstants.APRIL_TAG_LAYOUT.getFieldWidth(); // meters
    private static final double FIELD_LENGTH = FieldConstants.APRIL_TAG_LAYOUT.getFieldLength(); // meters

    private static final double shuttlingXOffset = 2.0;
    private static final double shuttlingYOffset = 2.5;

    public static enum Target {
        HUB,
        SHUTTLE_RIGHT,
        SHUTTLE_LEFT
    }

    private Target currentTarget = Target.HUB;

    private boolean overrideRedAlliance = false;
    private boolean overrideBlueAlliance = false;
    private final Supplier<SwerveDriveState> driveStateSupplier;
    
    public Targeting() {
        this(() -> RobotContainer.getDrive().getState());
    }

    Targeting(Supplier<SwerveDriveState> driveStateSupplier) {
        this.driveStateSupplier = driveStateSupplier;
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));
        Logger.recordOutput("Outreach/BasetballHoop", new Pose3d(FieldConstants.Outreach.basketballHoop, new Rotation3d()));

        SmartDashboard.putBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        SmartDashboard.putBoolean("OverrideRedAlliance", overrideRedAlliance);
    }

    /** Returns this cycle's compensated vector without recalculating the shot. */
    public static Translation2d differenceBetweenRobotAndTarget() {
        return compensatedToGoal;
    }

    // Called once per periodic update using pose and velocity from the same drivetrain state.
    void updateShotSolution(Pose2d robotPose, ChassisSpeeds robotRelSpeeds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelSpeeds.vxMetersPerSecond,
                robotRelSpeeds.vyMetersPerSecond,
                robotRelSpeeds.omegaRadiansPerSecond,
                robotPose.getRotation());
        var solution = MovingShotSolver.solve(
                robotPose, fieldSpeeds, targetPos, Turret.TURRET_OFFSET_FROM_CENTER);
        // Filtering the final vector delays compensation and changes launch geometry.
        compensatedToGoal = solution.toGoal();
        shotData = solution.shot();
        Logger.recordOutput("Targeting/StaticDistance", targetPos.getDistance(solution.pivotPosition()));
        Logger.recordOutput("Targeting/TurretPivot", solution.pivotPosition());
        Logger.recordOutput("Targeting/TurretPivotVelocity", solution.pivotVelocity());
        Logger.recordOutput("Targeting/ToGoalCompensation", compensatedToGoal);
        Logger.recordOutput("Targeting/TOF", solution.timeOfFlightS());
        Logger.recordOutput("Targeting/TimeOfFlight", solution.timeOfFlightS());
        Logger.recordOutput("Targeting/DistanceToTarget", compensatedToGoal.getNorm());
        Logger.recordOutput("Targeting/AngleToTargetDeg", shotData.turretAngleDeg());
        Logger.recordOutput("Targeting/TurretFeedforwardDegPerSec", shotData.turretFF_deg_s());
    }

    /**
     * Periodically calculate the shot data given the target position and robot movement
     */
    @Override
    public void periodic() {
        if (currentTarget == Target.HUB) {
            setPoseTargetingHub();
        } else if (currentTarget == Target.SHUTTLE_RIGHT) {
            setPoseTargetingShuttleRight();
        } else if (currentTarget == Target.SHUTTLE_LEFT) {
            setPoseTargetingShuttleLeft();
        }
        var driveState = driveStateSupplier.get();
        updateShotSolution(driveState.Pose, driveState.Speeds);
    }

    /**
     * Set the target of the targeting subsystem. This will change the target position
     */
    public void setTarget(Target target) {
        if (target == Target.HUB) {
            RobotContainer.getLEDController().switchPreset("hub");
        } else if (target == Target.SHUTTLE_LEFT  || target == Target.SHUTTLE_RIGHT) {
            RobotContainer.getLEDController().switchPreset("shuttle");
        }

        currentTarget = target;
    }

    /**
     * Set the target location to center of the hub 
     */
    private void setPoseTargetingHub() {

        overrideBlueAlliance = SmartDashboard.getBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        overrideRedAlliance = SmartDashboard.getBoolean("OverrideRedAlliance", overrideRedAlliance);

        if (overrideRedAlliance) {
            targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
            return;
        }

        if (overrideBlueAlliance) {
            targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
            return;
        }


        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
            }
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                targetPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
            }
        }
    }

    /**
     * Set the target location to 0, 0
     */
    private void setPoseTargetingShuttleRight() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, 0 + shuttlingYOffset);
        }
    }

    private void setPoseTargetingShuttleLeft() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetPos = new Translation2d(FIELD_LENGTH - shuttlingXOffset, 0 + shuttlingYOffset);
        } else {
            targetPos = new Translation2d(0 + shuttlingXOffset, FIELD_WIDTH - shuttlingYOffset);
        }
    }
    
    /**
     * Returns this cycle's base shot data, before shooter distance trim or manual overrides.
     * @return ShotCompensation.AdjustedShot shot data to use
     */
    public static ShotCompensation.AdjustedShot getShotData() {
        return shotData;
    }
}
