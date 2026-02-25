package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.pubsub.PubTopic;
import frc.lib.pubsub.SimpleDataPub;
import frc.lib.pubsub.impl.PubBroker;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class ShotCompensationCmd extends Command {
    private static final double NOMINAL_SHOT_TIME_S = 0.3; // see github issue #23 (https://github.com/Team293/Rebuilt/issues/23)

    private SimpleDataPub<ShotCompensation.AdjustedShot> publisher;

    private final CommandSwerveDrivetrain drive;
    private final Turret turret;

    public ShotCompensationCmd(CommandSwerveDrivetrain drive, Turret turret) {
        this.drive = drive;
        this.turret = turret;
    }

    @Override
    public void initialize() {
        publisher = PubBroker.getOrCreate(PubTopic.SHOT_COMPENSATION);
    }

    @Override
    public void execute() {
        // calculate the adjusted shot parameters based on the current robot movement and the turret's target position
        ShotCompensation.AdjustedShot targetAngleCompensated = ShotCompensation.compensateForMovement(
                drive.getPose(),
                drive.getState().Speeds,
                new Pose2d(turret.getTargetPos(), new Rotation2d()),
                NOMINAL_SHOT_TIME_S
        );

        // publish the compensated shot parameters for use by the turret aiming system
        publisher.publish(targetAngleCompensated);
    }
}
