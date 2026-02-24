package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.subsystem.BaseIO;
import frc.lib.subsystem.BaseInputClass;
import frc.lib.subsystem.IORefresher;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends BaseIO<TurretIO.TurretIOInputs>, IORefresher {

    @AutoLog
    public static class TurretIOInputs extends BaseInputClass {
        public double turretAngleDegrees = 0.0;
        public double pinionEncoder = 0.0;
        public double followerEncoder = 0.0;
        public double turretSetPointDegrees = 0.0;
        public Pose2d turretPosition = new Pose2d();
        public double robotOmegaDegPerSec = 0.0;
    }

    void setTurretAngle(double angle);
}
