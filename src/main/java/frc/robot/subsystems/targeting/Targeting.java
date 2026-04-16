package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private static Translation2d targetPos = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

    private static final double ROTATION_TOF_MULTIPLIER = 0.5;
    private static final double FIELD_WIDTH = 8.07; // meters
    private static final double FIELD_LENGTH = 16.54; // meters

    private static final double shuttlingXOffset = 2.0;
    private static final double shuttlingYOffset = 2.5;

    private static final MedianFilter distMedian = new MedianFilter(5);
    private static final LinearFilter distIIR = LinearFilter.singlePoleIIR(0.06, 0.02);

    private static final LinearFilter toGoalXFilter = LinearFilter.singlePoleIIR(0.04, 0.02);
    private static final LinearFilter toGoalYFilter = LinearFilter.singlePoleIIR(0.04, 0.02);

    public static enum Target {
        HUB,
        SHUTTLE_RIGHT,
        SHUTTLE_LEFT
    }

    private Target currentTarget = Target.HUB;

    private boolean overrideRedAlliance = false;
    private boolean overrideBlueAlliance = false;
    
    public Targeting() {
        Logger.recordOutput("HubTarget", FieldConstants.Hub.oppTopCenterPoint);
        Logger.recordOutput("ShuttleTarget", new Pose2d(0, 0, new Rotation2d()));

        SmartDashboard.putBoolean("OverrideBlueAlliance", overrideBlueAlliance);
        SmartDashboard.putBoolean("OverrideRedAlliance", overrideRedAlliance);
    }

    public static Translation2d differenceBetweenRobotAndTarget() {
        // calculate field-relative angle of the turret based on the turret motor position and the robot's heading
        // get pose of robo
        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d goalPose = targetPos;

        Translation2d turretPivotNow = Turret.TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

        double staticDistance = distIIR.calculate(distMedian.calculate(goalPose.getDistance(turretPivotNow)));
        double tof = ShotData.distanceToTOFConstant.get(staticDistance);
        Logger.recordOutput("Targeting/TOF", tof);
        // translate robot-center pose to the turret pivot location on the field

        ChassisSpeeds robotRelSpeeds = RobotContainer.getDrive().getState().Speeds;

        ChassisSpeeds speeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelSpeeds.vxMetersPerSecond,
                robotRelSpeeds.vyMetersPerSecond,
                robotRelSpeeds.omegaRadiansPerSecond,
                robotPose.getRotation()
            );

        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;

        Translation2d predictedRobotPos = robotPos.plus(
            new Translation2d(vx * tof, vy * tof)
        );

        Rotation2d predictedHeading =
            robotPose.getRotation().plus(
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * tof)
            );

        Translation2d predictedTurretPivot = Turret.TURRET_OFFSET_FROM_CENTER
            .rotateBy(predictedHeading)
            .plus(predictedRobotPos);

        Translation2d toGoalComp = goalPose.minus(predictedTurretPivot);

        Logger.recordOutput("Targeting/StaticDistance", staticDistance);
        Logger.recordOutput("Targeting/PredictedRobotPos", new Pose2d(predictedRobotPos, predictedHeading));
        Logger.recordOutput("Targeting/PredictedTurretPivot", new Pose2d(predictedTurretPivot, predictedHeading));
        Logger.recordOutput("Targeting/ToGoalCompensation", toGoalComp);
        Logger.recordOutput("Targeting/TimeOfFlight", tof);

        return new Translation2d(
            toGoalXFilter.calculate(toGoalComp.getX()),
            toGoalYFilter.calculate(toGoalComp.getY())
        );
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
     * Returns the current shot data
     * @return ShotCompensation.AdjustedShot shot data to use
     */
    public static ShotCompensation.AdjustedShot getShotData() {
        return shotData;
    }
}
