package frc.robot.subsystems.turret;

import frc.lib.FieldConstants;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.targeting.ShotCompensation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    public static final double TURRET_AIMING_TOLERANCE_DEGREES = 2.0; // degrees within which we consider the turret to be aimed at the target (+-)

    // HARDWARE CONSTANTS
    // gearing
    public static final double TURRET_GEAR_TEETH = 84.0; // number of teeth on fixed turret gear
    public static final double PINION_ENCODER_TEETH = 10.0; // number of teeth on pinion gear (driving turret)
    public static final double FOLLOWER_ENCODER_TEETH = 13.0; // number of teeth on follower gear
    public static final double TURRET_GEAR_RATIO = TURRET_GEAR_TEETH / PINION_ENCODER_TEETH; // gear ratio from motor to turret

    public static final double DEGREES_PER_REV = 360.0; // degrees in one revolution
    public static final double NORMALIZED_REVOLUTION = 1.0; // one full revolution in normalized units

    public static final double ENCODER_COMBINED_TEETH = PINION_ENCODER_TEETH * FOLLOWER_ENCODER_TEETH;
    public static final double ENCODER_COMBINED_PERIOD_REV = ENCODER_COMBINED_TEETH / PINION_ENCODER_TEETH;
    public static final double ENCODER_COMBINED_PERIOD_TURRET_REV =
        ENCODER_COMBINED_PERIOD_REV * (PINION_ENCODER_TEETH / TURRET_GEAR_TEETH);

    public static final double TURRET_CENTER_OFFSET_DEG = 144; // subtracted from robot relative heading
    public static final double TURRET_ROBOT_OFFSET_DEG = -61; // subtracted from robot relative heading to get turret relative heading

    public static final Translation2d TURRET_OFFSET_FROM_CENTER = new Translation2d(-0.3, 0); // distance from the center of the robot to the center of the turret, in meters 

    private TurretIO turretIO;

    public Turret() {
        super("Turret", new TurretIOInputsAutoLogged());
    }

    @Override
    public void onPeriodic() {
        ShotCompensation.AdjustedShot shotData = Targeting.getShotData();
        // turretIO.setTurretAngleFieldRelativeDegrees(0);

        // if (shotData != null) {
        //     double newTargetAngleDeg = shotData.turretAngleDeg();

        //     this.turretIO.setTurretAngleFieldRelativeDegrees(newTargetAngleDeg);
        // }

        this.turretIO.setTurretAngleFieldRelativeDegrees(getTurretAngleDegreesFieldRelative());


        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Pose2d turretTranslatedPose = new Pose2d(robotPose.getTranslation(), new Rotation2d(Math.toRadians(io.turretAngleDegreesFieldRelative)));
        Logger.recordOutput("Turret/RobotTurretPose", turretTranslatedPose);

        // Aiming ray: Pose2d[] from turret pivot to hub — renders as a path line in AdvantageScope
        Translation2d turretPivot = TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());
        Translation2d hub = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        Translation2d aimVec = hub.minus(turretPivot);
        Rotation2d aimAngle = aimVec.getAngle();

        final int NUM_POINTS = 6;
        Pose2d[] aimingRay = new Pose2d[NUM_POINTS];
        for (int i = 0; i < NUM_POINTS; i++) {
            double t = (double) i / (NUM_POINTS - 1);
            aimingRay[i] = new Pose2d(turretPivot.plus(aimVec.times(t)), aimAngle);
        }
        Logger.recordOutput("Turret/AimingRay", aimingRay);
    }

    public double getTurretAngleDegreesFieldRelative() {
        // calculate field-relative angle of the turret based on the turret motor position and the robot's heading
        // get pose of robo
        Pose2d robotPose = RobotContainer.getDrive().getPose();

        // translate robot-center pose to the turret pivot location on the field
        Translation2d turretPivot = TURRET_OFFSET_FROM_CENTER
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

        // vector from the turret pivot directly to the goal
        Translation2d goalPose = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        Translation2d toGoal = goalPose.minus(turretPivot);

        double angleToTarget = toGoal.getAngle().getDegrees();

        Logger.recordOutput("Turret/TurretPivot", new Pose2d(turretPivot, toGoal.getAngle()));
        Logger.recordOutput("Turret/AngleToTargetDeg", angleToTarget);
        return angleToTarget;
    }

    @Override
    protected Runnable setupDataRefresher() {
        turretIO = new TurretIOTalonFX(RobotContainer.getDrive());
        return useAsyncDataRefresher(turretIO);
    }

    /**
     * Checks if the turret is at the target angle, within the tolerance defined by TURRET_AIMING_TOLERANCE_DEGREES.
     * @return true if the turret is at the target angle, false otherwise
     */
    public boolean isAtTargetAngle() {
        double error =
            Math.abs(io.turretAngleDegreesFieldRelative - io.targetTurretDegrees);

        return error <= TURRET_AIMING_TOLERANCE_DEGREES;
    }
}
