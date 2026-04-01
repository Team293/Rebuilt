// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib.SpikeController;
import frc.robot.commands.EmptyHopper;
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
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveCmd = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        private final SwerveRequest.FieldCentricFacingAngle snapCmd = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(6, 0, 0.2);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new SpikeController(0, 0.05);
    private final CommandXboxController operatorController = new SpikeController(1, 0.05);

    public static CommandSwerveDrivetrain drive;
    private final Vision vision;
    private final Turret turret;
    //  private final Intake intake;
    private final Trigger trigger;
    private final Shooter shooter;
     private final Targeting targeting;
    private final Findexer findexer;

    public RobotContainer() {
        drive = TunerConstants.createDrivetrain();
        this.turret = new Turret();
        this.vision = new Vision(drive);
        // this.intake = new Intake(drive);
        this.shooter = new Shooter();
        this.targeting = new Targeting(drive);
        this.trigger = new Trigger(shooter, turret);
        this.findexer = new Findexer(trigger);

        configureBindings();

        NamedCommands.registerCommand(
            "ShootAtHub10s",
            Commands.defer(() -> new EmptyHopper(shooter, targeting, 10, true), Set.of(shooter, targeting))
        );

        NamedCommands.registerCommand(
            "ShootAtHub5s",
            Commands.defer(() -> new EmptyHopper(shooter, targeting, 5, true), Set.of(shooter, targeting))
        );

        NamedCommands.registerCommand(
            "ShuttleRightSide10s",
            Commands.defer(() -> new EmptyHopper(shooter, targeting, 10, false), Set.of(shooter, targeting))
        );

        autoChooser = drive.getAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);

        SignalLogger.stop();
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
            drive.applyRequest(() -> {

                double speedMultiplier = driverController.rightBumper().getAsBoolean() ? 0.4 : 1.0;

                double vx = -driverController.getLeftY() * MaxSpeed * speedMultiplier;
                double vy = -driverController.getLeftX() * MaxSpeed * speedMultiplier;

                // Snap angles 
                if (driverController.y().getAsBoolean()) { // Up
                    return snapCmd
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(Rotation2d.fromDegrees(0));
                } 
                else if (driverController.b().getAsBoolean()) { // Right
                    return snapCmd
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(Rotation2d.fromDegrees(270));
                } 
                else if (driverController.a().getAsBoolean()) { // Down
                    return snapCmd
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(Rotation2d.fromDegrees(180));
                } 
                else if (driverController.x().getAsBoolean()) { // Left
                    return snapCmd
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(Rotation2d.fromDegrees(90));
                }

                double angularMultiplier = driverController.rightBumper().getAsBoolean() ? 0.4 : 1.0;

                return driveCmd
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * angularMultiplier);
            })
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drive.applyRequest(() -> idle).ignoringDisable(true));

        driverController.leftStick().whileTrue(drive.applyRequest(() -> brake));
        driverController.rightStick().whileTrue(drive.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        driverController.leftBumper().onTrue(drive.runOnce(() -> drive.seedFieldCentric()));

        // operatorController.y().onTrue(vision.runOnce(() -> {
        // var estimatedPose = vision.getEstimatedPositionFromCameras();
        // if (estimatedPose != null) {
        // Logger.recordOutput("Vision/SnapshotEstimate", estimatedPose);
        // drive.resetPose(estimatedPose);
        // }
        // }));
    }

    private void setupIntakeBindings() {
        // toggle intake on A press
        // operatorController.rig-.onTrue(intake.runOnce(intake::switchDirection));
    }

    private void setupTargetingBindings() {
        operatorController.y().onTrue(targeting.runOnce(targeting::setTargetingHub));
        operatorController.x().onTrue(targeting.runOnce(targeting::setTargetingShuttleLeft));
        operatorController.b().onTrue(targeting.runOnce(targeting::setTargetingShuttleRight));
    }

    private void setupShooterBindings() {
        // toggle shooter on right trigger hold
        driverController.rightTrigger()
                .onTrue(shooter.runOnce(() -> shooter.setDriverRequestingShooting(true)))
                .onFalse(shooter.runOnce(() -> shooter.setDriverRequestingShooting(false)));
        driverController.leftTrigger()
                .onTrue(shooter.runOnce(() -> shooter.setRequestingWithForce(true)))
                .onFalse(shooter.runOnce(() -> shooter.setRequestingWithForce(false)));

        operatorController.rightStick().onTrue(shooter.runOnce(() -> shooter.zeroHood()));

        operatorController.leftBumper().onTrue(turret.runOnce(turret::toggleAimingOverride));

        operatorController.leftStick().onTrue(trigger.runOnce(() -> trigger.setReverseTrigger(true)));
        operatorController.leftStick().onFalse(trigger.runOnce(() -> trigger.setReverseTrigger(false)));

        operatorController.povUp().onTrue(shooter.runOnce(() -> shooter.changeDistanceTrim(0.1)));
        operatorController.povDown().onTrue(shooter.runOnce(() -> shooter.changeDistanceTrim(-0.1)));
        operatorController.povLeft().onTrue(turret.runOnce(() -> turret.changeTrim(2)));
        operatorController.povRight().onTrue(turret.runOnce(() -> turret.changeTrim(-2)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}