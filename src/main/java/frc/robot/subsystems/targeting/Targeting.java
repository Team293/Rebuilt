package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Targeting extends SubsystemBase {
    private static final double NOMINAL_SHOT_TIME_S = 0.3; // see github issue #23 (https://github.com/Team293/Rebuilt/issues/23)
    private static ShotCompensation.AdjustedShot shotData = new ShotCompensation.AdjustedShot(0.0, 0.0, 0.0, 0.0, 0.0);

    private final CommandSwerveDrivetrain drive;
    private final Turret turret;

    public Targeting(CommandSwerveDrivetrain drive, Turret turret) {
        this.drive = drive;
        this.turret = turret;
    }

    @Override
    public void periodic() {
        // calculate the adjusted shot parameters based on the current robot movement and the turret's target position
        shotData = ShotCompensation.compensateForMovement(
                drive.getPose(),
                drive.getState().Speeds,
                new Pose2d(turret.getTargetPos(), new Rotation2d()),
                NOMINAL_SHOT_TIME_S
        );
    }

    public static ShotCompensation.AdjustedShot getShotData() {
        return shotData;
    }
}
