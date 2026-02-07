package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.subsystem.IORefresher;
import frc.robot.CanID;
import frc.robot.subsystems.vision.photon.Camera;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class TurretIOSensorInputs implements TurretIO, IORefresher {
    private final Camera turretCamera;
    private final TalonFX turretMotor;
    private final DutyCycleEncoder enc11;
    private final DutyCycleEncoder enc13;

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> encoderSignal;

    private static final double kTurretGearRatio = 140/10;
    private static final double turretRotationOffset = -6.32;

    private static final int[] kValidTargetIDs = {9, 10, 23, 26};

    private Optional<Double> cachedYawErrorDeg = Optional.empty();
    private double cachedYawTimestampSec = Double.NEGATIVE_INFINITY;

    public TurretIOSensorInputs(Camera camera) {
        this.turretMotor = new TalonFX(CanID.TURRET_MOTOR.getID());
        Slot0Configs config = new Slot0Configs();
        config.kP = 1;
        config.kI = 0.01;
        config.kD = 0.3;
        config.kS = 0.194;
        config.kV = 0.1167;
        
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicAcceleration = 60;
        mm.MotionMagicCruiseVelocity = 20;
        mm.MotionMagicJerk = 0;

        this.turretMotor.getConfigurator().apply(mm);
        this.turretMotor.getConfigurator().apply(config);
        this.turretCamera = camera;
        this.enc11 = new DutyCycleEncoder(0);
        this.enc13 = new DutyCycleEncoder(1);

        this.encoderSignal = turretMotor.getPosition();
    }

    public void setTurretAngle(double angleDeg) {
        double rotations = (angleDeg / 360) * kTurretGearRatio;
        rotations = -rotations + turretRotationOffset;

        turretMotor.setControl(mmVoltage.withPosition(rotations));

        Logger.recordOutput("Turret/TargetRotations", rotations);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.enc11 = enc11.get();
        inputs.enc13 = enc13.get();
        double turretRotations = TurretMath.getTurretAngleRevs(inputs.enc13, inputs.enc11);
    
        inputs.turretAngleDegrees = TurretMath.toDegreesContinuous(turretRotations) - ((turretRotationOffset /14) * 360);
        
        Logger.recordOutput("Turret/CRTRevs", turretRotations);
        Logger.recordOutput("Turret/EncoderRevs", -((encoderSignal.getValueAsDouble() - turretRotationOffset) / 14) * 360);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(encoderSignal);
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
