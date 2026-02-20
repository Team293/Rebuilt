package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends BaseIO<TurretIO.TurretIOInputs> {

    @AutoLog
    public static class TurretIOInputs extends BaseInputClass {
        public double turretAngleDegrees = 0.0;
        public double pinionEncoder = 0.0;
        public double followerEncoder = 0.0;
    }

    double getAngleOffsetFromPose(Translation2d robotPose, Translation2d target);
}
