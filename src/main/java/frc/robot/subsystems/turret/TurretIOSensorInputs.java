package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.AngleLowPassFilter;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;
import frc.robot.subsystems.vision.photon.Camera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class TurretIOSensorInputs implements TurretIO, IORefresher {
    private final Camera turretCamera;
    private final TalonFX turretMotor;
    private final DutyCycleEncoder enc11;
    private final DutyCycleEncoder enc13;

    private final AngleLowPassFilter turretAngleFilter =
            new AngleLowPassFilter(0.15);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0.0);

    private static final double kTurretGearRatio = 100.0; // 100:1, replace with real value
    private static final double kTurretMinAngleDegrees = -180.0;
    private static final double kTurretMaxAngleDegrees = 180.0;

    private static final int[] kValidTargetIDs = {9, 10, 23, 26};

    private Optional<Double> cachedYawErrorDeg = Optional.empty();
    private double cachedYawTimestampSec = Double.NEGATIVE_INFINITY;

    public TurretIOSensorInputs(Camera camera) {
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = 20;
        mm.MotionMagicAcceleration = 60;
        mm.MotionMagicJerk = 0;

        this.turretMotor.getConfigurator().apply(mm);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.NeutralMode = NeutralModeValue.Brake;

        turretMotor.getConfigurator().apply(motorOutput);

        this.turretCamera = camera;
        this.enc11 = new DutyCycleEncoder(0);
        this.enc13 = new DutyCycleEncoder(1);
    }

    public void setTurretAngle(double angleDeg) {
        double clampedDeg = Math.max(kTurretMinAngleDegrees, Math.min(kTurretMaxAngleDegrees, angleDeg));
        double motorRotations = (clampedDeg / 360.0) * kTurretGearRatio;
        turretMotor.setControl(mmRequest.withPosition(motorRotations));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.enc11 = enc11.get();
        inputs.enc13 = enc13.get();
        double turretRotations = TurretMath.getTurretPosition(inputs.enc13, inputs.enc11);
        double rawAngle = TurretMath.positionToDegrees(turretRotations);
        inputs.turretAngleDegrees =
                turretAngleFilter.calculate(rawAngle);
    }

    @Override
    public void refreshData() {
        cachedYawErrorDeg = Optional.empty();

        if (turretCamera == null || turretCamera.getPhotonCamera() == null) return;

        List<PhotonPipelineResult> unread = turretCamera.getPhotonCamera().getAllUnreadResults();
        if (unread.isEmpty()) return;

        PhotonPipelineResult latest = null;
        double latestTs = cachedYawTimestampSec;

        for (PhotonPipelineResult r : unread) {
            double ts = r.getTimestampSeconds();
            if (ts > latestTs) {
                latestTs = ts;
                latest = r;
            }
        }

        if (latest == null || !latest.hasTargets()) return;

        var validTargets = latest.getTargets().stream()
                .filter(t -> {
                    int id = t.getFiducialId();
                    for (int valid : kValidTargetIDs) {
                        if (id == valid) return true;
                    }
                    return false;
                })
                .toList();
        if (validTargets.isEmpty()) return;

        double lowestAmbiguity = Double.MAX_VALUE;
        double yawDeg = 0.0;

        for (var t : validTargets) {
            double amb = t.getPoseAmbiguity();
            if (amb < lowestAmbiguity) {
                lowestAmbiguity = amb;
                yawDeg = t.getYaw();
            }
        }

        cachedYawErrorDeg = Optional.of(yawDeg);
        cachedYawTimestampSec = latestTs;
    }

    @Override
    public double getAngleOffsetFromPose(Translation2d robotPose, Translation2d target) {
        double angleToTargetDeg = Math.toDegrees(
                Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX())
        );

        return MathUtil.inputModulus(angleToTargetDeg, -180.0, 180.0);
    }

    @Override
    public Optional<Double> getErrorFromCamera() {
        return cachedYawErrorDeg;
    }
}
