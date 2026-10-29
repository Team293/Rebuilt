package frc.robot.subsystems.targeting;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class TargetingTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void unregisterSubsystems() {
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    void consumersShareOneSolutionUntilNextUpdate() {
        Targeting targeting = new Targeting();
        targeting.updateShotSolution(
            new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(30)),
            new ChassisSpeeds(1.5, -0.5, 0.8));

        Translation2d turretVector = Targeting.differenceBetweenRobotAndTarget();
        var shot = Targeting.getShotData();
        for (int i = 0; i < 20; i++) {
            // Shooter, turret, and telemetry reads must not advance any filters.
            assertSame(turretVector, Targeting.differenceBetweenRobotAndTarget());
            assertSame(shot, Targeting.getShotData());
        }
        assertEquals(turretVector.getAngle().getDegrees(), shot.turretAngleDeg(), 1e-9);
        assertEquals(ShotData.distanceToRPM.get(turretVector.getNorm()), shot.rpm(), 1e-9);
        assertEquals(ShotData.distanceToHoodAngle.get(turretVector.getNorm()), shot.hoodAngleDeg(), 1e-9);

        targeting.updateShotSolution(new Pose2d(8.0, 1.0, new Rotation2d()), new ChassisSpeeds());
        assertNotEquals(turretVector, Targeting.differenceBetweenRobotAndTarget());
        assertNotSame(shot, Targeting.getShotData());
    }

    @Test
    void schedulerPublishesOnceAfterVisionAndBeforeConsumers() {
        List<String> order = new ArrayList<>();
        AtomicInteger samples = new AtomicInteger();
        SwerveDriveState state = new SwerveDriveState();
        state.Speeds = new ChassisSpeeds(1.0, 0.5, 0.2);
        new SubsystemBase() {
            @Override
            public void periodic() {
                order.add("vision");
                state.Pose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45));
            }
        };
        new Targeting(() -> {
            order.add("targeting");
            samples.incrementAndGet();
            assertEquals(3.0, state.Pose.getX());
            return state;
        });
        List<Translation2d> consumerVectors = new ArrayList<>();
        for (String consumer : List.of("turret", "shooter")) {
            new SubsystemBase() {
                @Override
                public void periodic() {
                    order.add(consumer);
                    consumerVectors.add(Targeting.differenceBetweenRobotAndTarget());
                }
            };
        }

        CommandScheduler.getInstance().run();
        assertEquals(List.of("vision", "targeting", "turret", "shooter"), order);
        assertEquals(1, samples.get());
        assertSame(consumerVectors.get(0), consumerVectors.get(1));
        Translation2d firstCycle = consumerVectors.get(0);

        order.clear();
        consumerVectors.clear();
        CommandScheduler.getInstance().run();
        assertEquals(2, samples.get());
        assertSame(consumerVectors.get(0), consumerVectors.get(1));
        assertNotSame(firstCycle, consumerVectors.get(0));
    }
}
