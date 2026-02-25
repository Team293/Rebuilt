// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.ShotCompensationCmd;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.vision.Vision;

import java.util.HashSet;
import java.util.Set;

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

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final Set<Command> alwaysRunCommands = new HashSet<>();

    public static CommandSwerveDrivetrain drive;
    private final Vision vision;
    private final Turret turret;
    private final Intake intake;
    private final Indexer indexer;
    private final Shooter shooter;

    public RobotContainer() {
        drive = TunerConstants.createDrivetrain();
        turret = new Turret(drive);
        autoChooser = drive.getAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);
        
        this.vision = new Vision(drive);
        this.intake = new Intake(drive);
        this.indexer = new Indexer(2);
        this.shooter = new Shooter();

        alwaysRunCommands.add(
                new ShotCompensationCmd(drive, this.turret)
        );

        configureBindings();
    }

    public static CommandSwerveDrivetrain getDrive() {
        return drive;
    }

    private void configureBindings() {
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

        operatorController.a().onTrue(intake.run(() -> intake.deploy()));
        operatorController.b().onTrue(intake.run(() -> intake.retract()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Set<Command> getAlwaysRunCommands() {
        return alwaysRunCommands;
    }
}
