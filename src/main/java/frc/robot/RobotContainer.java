// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.FieldConstants;
import frc.lib.SpikeController;
import frc.lib.led.LEDController;
import frc.lib.led.LEDPreset;
import frc.robot.commands.SetFlywheelState;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RequestShootingForSeconds;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.findexer.Findexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.targeting.Targeting.Target;
import frc.robot.subsystems.trigger.Trigger;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;

@SuppressWarnings("unused")
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

    private final CommandXboxController driverController = new SpikeController(0, 0.05);
    private final CommandXboxController operatorController = new SpikeController(1, 0.05);

    public static CommandSwerveDrivetrain drive;
    private final Vision vision;
    private final Turret turret;
    private final Intake intake;
    private final Trigger trigger;
    private final Shooter shooter;
    private final Targeting targeting;
    private final Findexer findexer;

    private static LEDController ledController;

    public RobotContainer() {
        drive = TunerConstants.createDrivetrain();
        ledController = new LEDController(1);
        this.vision = new Vision(drive);
        // Subsystem periodic methods run in registration order: apply vision first,
        // then publish one targeting solution before either aiming subsystem reads it.
        this.targeting = new Targeting();
        this.turret = new Turret(operatorController, targeting::getSolution);
        this.intake = new Intake();
        this.shooter = new Shooter(operatorController, targeting::getSolution);
        this.trigger = new Trigger(shooter, turret);
        this.findexer = new Findexer(trigger);
        
        NamedCommands.registerCommand("enableIntake", new SetIntakeState(intake, true));
        NamedCommands.registerCommand("disableIntake", new SetIntakeState(intake, false));
        NamedCommands.registerCommand("startFlywheel", new SetFlywheelState(shooter, true));
        NamedCommands.registerCommand("stopFlywheel", new SetFlywheelState(shooter, false));
        NamedCommands.registerCommand("requestShooting5S", new RequestShootingForSeconds(shooter, true, 5.0));
        NamedCommands.registerCommand("requestShooting6S", new RequestShootingForSeconds(shooter, true, 6.0));
        NamedCommands.registerCommand("requestShooting7S", new RequestShootingForSeconds(shooter, true, 7.0));
        NamedCommands.registerCommand("requestShooting8S", new RequestShootingForSeconds(shooter, true, 8.0));
        NamedCommands.registerCommand("requestShooting9S", new RequestShootingForSeconds(shooter, true, 9.0));
        NamedCommands.registerCommand("requestShooting10S", new RequestShootingForSeconds(shooter, true, 10.0));
        NamedCommands.registerCommand("requestShooting11S", new RequestShootingForSeconds(shooter, true, 11.0));
        NamedCommands.registerCommand("requestShooting12S", new RequestShootingForSeconds(shooter, true, 12.0));
        NamedCommands.registerCommand("requestShooting15S", new RequestShootingForSeconds(shooter, true, 15.0));
        NamedCommands.registerCommand("requestShooting20S", new RequestShootingForSeconds(shooter, true, 20.0));
        NamedCommands.registerCommand("stopShooting", new RequestShootingForSeconds(shooter, false, 0.0));
        NamedCommands.registerCommand("deployIntake", new DeployIntake(intake));
        NamedCommands.registerCommand("targetHub", targeting.runOnce(() -> targeting.setTarget(Target.HUB)));
        NamedCommands.registerCommand("requestShooting3S", new RequestShootingForSeconds(shooter, true, 3.0));
        NamedCommands.registerCommand("requestShooting4S", new RequestShootingForSeconds(shooter, true, 4.0));
        NamedCommands.registerCommand("requestShooting2S", new RequestShootingForSeconds(shooter, true, 2.0));
        NamedCommands.registerCommand("requestShooting2_5S", new RequestShootingForSeconds(shooter, true, 2.5));
        NamedCommands.registerCommand("requestShooting6_25S", new RequestShootingForSeconds(shooter, true, 6.25));
        NamedCommands.registerCommand("collectFuel", Commands.startEnd(
                intake::enable, intake::disable, intake));
        NamedCommands.registerCommand("autoFlywheel", Commands.startEnd(
                () -> shooter.setDriverSpinUpFlywheel(true),
                () -> shooter.setDriverSpinUpFlywheel(false), shooter));
        NamedCommands.registerCommand("autoIntakeCleanup", Commands.startEnd(
                intake::disable, intake::disable, intake));
        NamedCommands.registerCommand("holdAutoPosition", drive.applyRequest(() -> brake));
        // The path sequence owns drive; this parallel cleanup brakes if the deadline interrupts it.
        NamedCommands.registerCommand("autoDriveCleanup", Commands.startEnd(
                () -> {}, () -> drive.setControl(brake)));

        // These stationary autos have no path from which PathPlanner can infer a start pose.
        // The configured 32.5-inch robot starts with its front bumper on the line.
        double startingX = FieldConstants.LinesVertical.starting - Inches.of(16.25).in(Meters);
        NamedCommands.registerCommand("resetLeftBumpStart", AutoBuilder.resetOdom(new Pose2d(
                startingX, FieldConstants.LinesHorizontal.leftBumpEnd + FieldConstants.LeftBump.width / 2,
                Rotation2d.kZero)));
        NamedCommands.registerCommand("resetRightBumpStart", AutoBuilder.resetOdom(new Pose2d(
                startingX, FieldConstants.LinesHorizontal.rightBumpStart - FieldConstants.RightBump.width / 2,
                Rotation2d.kZero)));

        autoChooser = drive.getAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);
        SmartDashboard.putNumber("Field Length", FieldConstants.APRIL_TAG_LAYOUT.getFieldLength());
        SmartDashboard.putNumber("Field Width", FieldConstants.APRIL_TAG_LAYOUT.getFieldWidth());

        configureBindings();
        configureLEDPresets();

        ledController.switchPreset("hub");
    }

    private void configureLEDPresets() {
        ledController.registerPreset(
            new LEDPreset("hub", -0.25)
        );

        ledController.registerPreset(
            new LEDPreset("shuttle", -0.23)
        );

        ledController.registerPreset(
            new LEDPreset("fixed", -0.21)
        );
    }

    public static LEDController getLEDController() {
        return ledController;
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

                double speedMultiplier = driverController.rightBumper().getAsBoolean() ? 0.30 : 1.0;

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

                double angularMultiplier = driverController.rightBumper().getAsBoolean() ? 0.5 : 1.0;

                double flipControls = driverController.leftStick().getAsBoolean() ? -1 : 1;   

                return driveCmd
                    .withVelocityX(vx * flipControls)
                    .withVelocityY(vy * flipControls)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * angularMultiplier * flipControls);
            })


        );
        driverController.povLeft().onTrue(drive.runOnce(drive::resetAllianceHeadingToAllianceForward));
        driverController.povDown().onTrue(drive.runOnce(() ->
            drive.resetRotation(vision.getAprilTagHeading().orElseGet(drive::getRotation))));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drive.applyRequest(() -> idle).ignoringDisable(true));

        driverController.leftTrigger().whileTrue(drive.applyRequest(() -> brake));
    }

    private void setupIntakeBindings() {
        // toggle intake on A press
        driverController.leftBumper().onTrue(intake.runOnce(intake::toggle));
        driverController.povUp().onTrue(intake.runOnce(intake::switchDirection));
    }

    private void setupTargetingBindings() {
        operatorController.start().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.AUTO)));
        operatorController.y().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.HUB)));
        operatorController.x().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.SHUTTLE_LEFT)));
        operatorController.b().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.SHUTTLE_RIGHT)));
    }

    public void enableAutomaticTargeting() {
        targeting.setTarget(Target.AUTO);
    }

    private void setupShooterBindings() {
        // toggle shooter on right trigger hold
        operatorController.rightTrigger()
                .onTrue(shooter.runOnce(() -> shooter.setDriverSpinUpFlywheel(true)))
                .onFalse(shooter.runOnce(() -> shooter.setDriverSpinUpFlywheel(false)));
        operatorController.leftTrigger()
                .onTrue(shooter.runOnce(() -> shooter.setActuateHoodAndLaunch(true)))
                .onFalse(shooter.runOnce(() -> shooter.setActuateHoodAndLaunch(false)));
        
        operatorController.leftBumper()
                .onTrue(shooter.runOnce(() -> shooter.setActuateHoodAndLaunch(true, true)))
                .onFalse(shooter.runOnce(() -> shooter.setActuateHoodAndLaunch(false, false)));

        var operatorFeedHeld = operatorController.leftTrigger().or(operatorController.leftBumper());
        // Latch the warning until release, even if shooter readiness changes after the jam.
        operatorFeedHeld.and(findexer::isJamLockedOut).and(DriverStation::isTeleopEnabled)
                .onTrue(Commands.startEnd(
                        () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
                        () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                        .until(() -> !operatorFeedHeld.getAsBoolean() || !DriverStation.isTeleopEnabled())
                        .ignoringDisable(true)
                        .withName("Findexer jam warning"));

        operatorController.rightStick().onTrue(shooter.runOnce(() -> shooter.zeroHood()));

        operatorController.rightBumper().onTrue(turret.runOnce(turret::toggleAimingOverride));

        operatorController.leftStick().onTrue(trigger.runOnce(() -> trigger.setReverseTrigger(true)));
        operatorController.leftStick().onFalse(trigger.runOnce(() -> trigger.setReverseTrigger(false)));

        operatorController.povUp().onTrue(shooter.runOnce(() -> shooter.changeDistanceTrim(0.1)));
        operatorController.povDown().onTrue(shooter.runOnce(() -> shooter.changeDistanceTrim(-0.1)));

        operatorController.a().onTrue(shooter.runOnce(() -> shooter.setDistanceTrim(0)));
        operatorController.povLeft().onTrue(turret.runOnce(() -> turret.changeTrim(1)));
        operatorController.povRight().onTrue(turret.runOnce(() -> turret.changeTrim(-1)));

        driverController.leftTrigger().onTrue(shooter.runOnce(() -> shooter.setOverrideStopShooting(true)));
        driverController.leftTrigger().onFalse(shooter.runOnce(() -> shooter.setOverrideStopShooting(false)));
    }

    public Command getAutonomousCommand() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            String auto = System.getProperty("simAuto", "");
            String path = System.getProperty("simPath", "");
            if (!auto.isBlank()) return new PathPlannerAuto(auto);
            if (!path.isBlank()) return drive.createPathCommand(path);
        }
        return autoChooser.getSelected();
    }

    public void stopAutoOutputs() {
        intake.disable();
        shooter.setActuateHoodAndLaunch(false);
        shooter.setDriverSpinUpFlywheel(false);
        drive.setControl(brake);
    }

    public void updateDriverFeedback() {
        double rumble = DriverStation.isTeleopEnabled() ? turret.getDirectionalWraparoundRumble() : 0.0;
        driverController.getHID().setRumble(RumbleType.kLeftRumble, Math.max(0.0, rumble));
        driverController.getHID().setRumble(RumbleType.kRightRumble, Math.max(0.0, -rumble));
    }
}
