package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import frc.lib.FieldConstants;
import frc.lib.state.StateMachine;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private static final double kTurretDeadbandDeg = 2.5; // degrees the turret must be within to be "on target"

    private TurretIOSensorInputs sensorData;
    private final CommandSwerveDrivetrain drive;

    private enum State { IDLE, TARGETING_HUB, TARGETING_SHUTTLE, ZEROING, MANUAL_CONTROL }

    private final StateMachine<State> tsm;

    private final TurretRequest turretRequest = new TurretRequest();

    public static class TurretRequest {
        public double targetAngleDegrees;
    }

    public Turret(CommandSwerveDrivetrain drive) {
        super("Turret", new TurretIO.TurretIOInputs());
        Logger.recordOutput("Hub/Center", FieldConstants.Hub.innerCenterPoint);
        this.drive = drive;
        this.tsm =
                StateMachine.forEnum(State.class)
                        .initial(State.IDLE)
                        .state(State.IDLE, cfg -> cfg
                                .onEnter(() -> {
                                    // noop
                                })
                        )

                        .state(State.TARGETING_HUB, cfg -> cfg
                                .onEnter(() -> {
                                    double turretAngle = io.turretAngleDegrees;
                                    double offset = sensorData.getAngleOffsetFromPose(
                                            drive.getPose().getTranslation(),
                                            FieldConstants.Hub.innerCenterPoint.toTranslation2d()
                                    );
                                    turretRequest.targetAngleDegrees = turretAngle + offset; // == angleToHub
                                    sensorData.setTurretAngle(turretRequest.targetAngleDegrees);
                                })
                                .onTick(() -> {
                                    double turretAbsDeg = io.turretAngleDegrees;
                                    double poseOffsetDeg = sensorData.getAngleOffsetFromPose(
                                            drive.getPose().getTranslation(),
                                            FieldConstants.Hub.innerCenterPoint.toTranslation2d()
                                    );

                                    double targetDeg = getTargetDeg(turretAbsDeg, poseOffsetDeg);
                                    turretRequest.targetAngleDegrees = targetDeg;
                                    sensorData.setTurretAngle(targetDeg);
                                })
                        )
                        .build();
    }

    private double getTargetDeg(double turretAbsDeg, double poseOffsetDeg) {
        double targetDeg = turretAbsDeg + poseOffsetDeg; // == angleToHub

        var yawOpt = sensorData.getErrorFromCamera();
        if (yawOpt.isPresent()) {
            double yawErrDeg = yawOpt.get();

            if (Math.abs(yawErrDeg) > kTurretDeadbandDeg) {
                targetDeg = targetDeg - yawErrDeg;
            }
        }

        targetDeg = MathUtil.inputModulus(targetDeg, -180.0, 180.0);
        return targetDeg;
    }

    @Override
    public void onPeriodic() {
        tsm.tick();
    }

    @Override
    protected Runnable setupDataRefresher() {
        sensorData = new TurretIOSensorInputs(null);
        return useAsyncDataRefresher(sensorData);
    }
}
