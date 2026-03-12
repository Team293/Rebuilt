// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib.SpikeController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.findexer.Findexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.trigger.Trigger;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveCmd = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new SpikeController(0, 0.05);
    private final CommandXboxController operatorController = new SpikeController(1, 0.05);

    public static CommandSwerveDrivetrain drive;
    private final Vision vision;
    private final Turret turret;
    // private final Intake intake;
    private final Trigger trigger;
    private final Shooter shooter;
    // private final Targeting targeting;
    // private final Findexer findexer;

    public RobotContainer() {
        drive = TunerConstants.createDrivetrain();
        this.turret = new Turret();
        this.vision = new Vision(drive);
        // this.intake = new Intake(drive);
        this.shooter = new Shooter();
        // this.targeting = new Targeting(drive);
        this.trigger = new Trigger(shooter, turret);
        // this.findexer = new Findexer(trigger);

        autoChooser = drive.getAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);

        configureBindings();
    }

    public static CommandSwerveDrivetrain getDrive() {
        return drive;
    }

    private void configureBindings() {
        setupSwerveBindings();
        setupIntakeBindings();
        setupTargetingBindings();
        setupShooterBindings();
    }

    private void setupSwerveBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drive.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drive.applyRequest(() ->
                        driveCmd.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drive.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drive.applyRequest(() -> brake));
        driverController.b().whileTrue(drive.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        
        driverController.leftBumper().onTrue(drive.runOnce(() -> drive.seedFieldCentric()));

        // operatorController.y().onTrue(vision.runOnce(() -> {
        //     var estimatedPose = vision.getEstimatedPositionFromCameras();
        //     if (estimatedPose != null) {
        //         Logger.recordOutput("Vision/SnapshotEstimate", estimatedPose);
        //         drive.resetPose(estimatedPose);
        //     }
        // }));
    }

    private void setupIntakeBindings() {
        // toggle intake on B press
        // operatorController.b().onTrue(intake.run(intake::toggleIntake));
    }

    private void setupTargetingBindings() {
        // operatorController.rightBumper().onTrue(targeting.run(targeting::setTargetingHub));
        // operatorController.leftBumper().onTrue(targeting.run(targeting::setTargetingShuttle));
    }

    private void setupShooterBindings() {
        // toggle shooter on right trigger hold
        driverController.rightBumper()
                .onTrue(shooter.run(() -> shooter.setDriverRequestingShooting(true)))
                .onFalse(shooter.run(() -> shooter.setDriverRequestingShooting(false)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}