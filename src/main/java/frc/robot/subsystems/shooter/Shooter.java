package frc.robot.subsystems.shooter;

import frc.lib.pubsub.PubResult;
import frc.lib.pubsub.PubTopic;
import frc.lib.pubsub.SimpleDataPub;
import frc.lib.pubsub.impl.PubBroker;
import frc.lib.subsystem.SpikeSystem;
import frc.robot.subsystems.turret.calc.ShotCompensation;

import java.util.Optional;

public class Shooter extends SpikeSystem<ShooterIO.ShooterIOInputs> {
    private final SimpleDataPub<ShotCompensation.AdjustedShot> shotCompensationPub;

    private ShooterIO shooterIO;

    public Shooter() {
        super("Shooter", new ShooterIO.ShooterIOInputs());
        shotCompensationPub = PubBroker.getOrCreate(PubTopic.SHOT_COMPENSATION);
    }

    @Override
    public void onPeriodic() {
        Optional<PubResult<ShotCompensation.AdjustedShot>> pollRes = shotCompensationPub.poll();

        pollRes.ifPresent(res -> {
            if (!res.isInTimestamp()) {
                // we can add rejection logic if needed
                System.out.println("WARNING: Shooter compensation data is stale!");
            }

            ShotCompensation.AdjustedShot compensatedShot = res.getData();
            shooterIO.setHoodAngle(compensatedShot.hoodAngleDeg());
            shooterIO.setFlywheelVelocity(compensatedShot.rpm() / 60.0);
        });
    }

    @Override
    protected Runnable setupDataRefresher() {
        shooterIO = new ShooterIOTalonFX();
        return useAsyncDataRefresher(shooterIO);
    }
}
