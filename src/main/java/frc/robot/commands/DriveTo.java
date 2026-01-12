package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class DriveTo extends Command {
    private CommandSwerveDrivetrain drive;
    private Pose2d target;

    private PPHolonomicDriveController controller;

    private PathPlannerTrajectoryState trajecState;

    private static final double kTreshM = 0.01;

    public DriveTo(
            CommandSwerveDrivetrain drive,
            Pose2d target
    ) {
        this.drive = drive;
        this.target = target;
        this.controller = new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(4, 0, 0),
                0.02); // loop time

        this.trajecState = new PathPlannerTrajectoryState();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        trajecState.pose = target;
    }

    @Override
    public boolean isFinished() {
        double dx = drive.getPose().getX() - target.getX();
        double dy = drive.getPose().getY() - target.getY();
        double distanceError = Math.hypot(dx, dy);

        double rotationError = Math.abs(
                drive.getRotation().minus(target.getRotation()).getRadians()
        );

        return distanceError <= kTreshM &&
                rotationError <= Units.degreesToRadians(0.5);
    }

    @Override
    public void execute() {
        ChassisSpeeds runSpeeds = controller.calculateRobotRelativeSpeeds(
                drive.getPose(),
                trajecState
        );

        drive.applyRequest(() -> {
            SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
            request.Speeds = runSpeeds;
            return request;
        });
    }

    @Override
    public void end(boolean interrupted) {
        drive.applyRequest(SwerveRequest.Idle::new); // it might be brake, im not too sure, needs testing
    }
}
