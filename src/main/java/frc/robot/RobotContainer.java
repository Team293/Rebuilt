// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
        this.turret = new Turret(operatorController);
        this.vision = new Vision(drive);
        this.intake = new Intake();
        this.shooter = new Shooter(operatorController);
        this.targeting = new Targeting();
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


        autoChooser = drive.getAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);

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

                double speedMultiplier = driverController.rightBumper().getAsBoolean() ? 0.2 : 1.0;

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
        operatorController.y().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.HUB)));
        operatorController.x().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.SHUTTLE_LEFT)));
        operatorController.b().onTrue(targeting.runOnce(() -> targeting.setTarget(Target.SHUTTLE_RIGHT)));
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
        return autoChooser.getSelected();
    }
}