package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.littletonrobotics.junction.LogTable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionIOPhotonCameraTest {
    private static PhotonTrackedTarget target(double distance, double ambiguity) {
        var target = new PhotonTrackedTarget();
        target.bestCameraToTarget = new Transform3d(distance, 0, 0, new Rotation3d());
        target.poseAmbiguity = ambiguity;
        return target;
    }

    private static EstimatedRobotPose estimate(double timestamp, List<PhotonTrackedTarget> targets) {
        return new EstimatedRobotPose(new Pose3d(timestamp, 1, 0, new Rotation3d()), timestamp,
                targets, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }

    @Test
    void retainsCompleteBatchesAndCopiesPhotonDataBeforePublication() {
        var target = target(2, 0.1);
        var targets = new ArrayList<>(List.of(target, target(4, 0.15)));
        var source = new AtomicReference<>(List.of(estimate(10, targets)));
        var camera = new VisionIOPhotonCamera(Pose2d::new, ignored -> source.get());
        camera.refreshData();

        // Photon result lists and targets may change after the worker publishes them.
        targets.clear();
        target.poseAmbiguity = 0.9;
        target.bestCameraToTarget = new Transform3d(100, 0, 0, new Rotation3d());
        source.set(List.of(estimate(11, List.of(target(1, 0.05)))));
        camera.refreshData();
        source.set(List.of());
        camera.refreshData(); // An empty poll must not overwrite unread observations.

        var inputs = new VisionIOInputsAutoLogged();
        camera.updateInputs(inputs);
        assertArrayEquals(new double[] {10, 11}, inputs.timestampsSeconds);
        assertArrayEquals(new int[] {2, 1}, inputs.tagCounts);
        assertArrayEquals(new double[] {3, 1}, inputs.averageTagDistancesMeters);
        assertArrayEquals(new double[] {0.15, 0.05}, inputs.maxTagAmbiguities);
        assertEquals(10, inputs.estimatedRobotPoses[0].getX());
        assertEquals(11, inputs.estimatedRobotPoses[1].getX());

        var table = new LogTable(0);
        inputs.toLog(table);
        var replay = new VisionIOInputsAutoLogged();
        replay.fromLog(table);
        assertArrayEquals(inputs.estimatedRobotPoses, replay.estimatedRobotPoses);
        assertArrayEquals(inputs.timestampsSeconds, replay.timestampsSeconds);
        assertArrayEquals(inputs.tagCounts, replay.tagCounts);
        assertArrayEquals(inputs.averageTagDistancesMeters, replay.averageTagDistancesMeters);
        assertArrayEquals(inputs.maxTagAmbiguities, replay.maxTagAmbiguities);

        camera.updateInputs(inputs);
        assertEquals(0, inputs.estimatedRobotPoses.length);
        assertEquals(0, inputs.timestampsSeconds.length);
        assertEquals(0, inputs.tagCounts.length);
        assertEquals(0, inputs.averageTagDistancesMeters.length);
        assertEquals(0, inputs.maxTagAmbiguities.length);
    }

    @Test
    void mainThreadCanDrainPreviousBatchWhileCameraProcessingIsBlocked() throws Exception {
        var processing = new CountDownLatch(1);
        var finishProcessing = new CountDownLatch(1);
        var calls = new AtomicInteger();
        var camera = new VisionIOPhotonCamera(Pose2d::new, ignored -> {
            if (calls.getAndIncrement() == 0) return List.of(estimate(1, List.of()));
            processing.countDown();
            try {
                if (!finishProcessing.await(2, TimeUnit.SECONDS)) throw new AssertionError("Worker timed out");
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new AssertionError(e);
            }
            return List.of(estimate(2, List.of()), estimate(3, List.of()));
        });
        camera.refreshData();
        var worker = Executors.newSingleThreadExecutor();
        try {
            var refresh = worker.submit(camera::refreshData);
            assertTrue(processing.await(2, TimeUnit.SECONDS));
            var inputs = new VisionIOInputsAutoLogged();
            assertTimeoutPreemptively(Duration.ofSeconds(1), () -> camera.updateInputs(inputs));
            assertArrayEquals(new double[] {1}, inputs.timestampsSeconds);

            finishProcessing.countDown();
            refresh.get(2, TimeUnit.SECONDS);
            // Publication must not alter the snapshot already logged for this cycle.
            assertArrayEquals(new double[] {1}, inputs.timestampsSeconds);
            camera.updateInputs(inputs);
            assertArrayEquals(new double[] {2, 3}, inputs.timestampsSeconds);
            camera.updateInputs(inputs);
            assertEquals(0, inputs.timestampsSeconds.length);
        } finally {
            finishProcessing.countDown();
            worker.shutdownNow();
            assertTrue(worker.awaitTermination(2, TimeUnit.SECONDS));
        }
    }

    @Test
    @Timeout(10)
    void concurrentPublicationAndDrainNeverSplitDuplicateOrLoseBatches() throws Exception {
        var batchNumber = new AtomicInteger();
        var camera = new VisionIOPhotonCamera(Pose2d::new, ignored -> {
            int firstTimestamp = batchNumber.getAndIncrement() * 2;
            return List.of(estimate(firstTimestamp, List.of(target(1, 0.1))),
                    estimate(firstTimestamp + 1, List.of(target(2, 0.15))));
        });
        var worker = Executors.newSingleThreadExecutor();
        try {
            var refresh = worker.submit(() -> {
                for (int i = 0; i < 1000; i++) camera.refreshData();
            });
            var inputs = new VisionIOInputsAutoLogged();
            int received = 0;
            while (true) {
                boolean finished = refresh.isDone();
                camera.updateInputs(inputs);
                assertEquals(0, inputs.timestampsSeconds.length % 2);
                assertEquals(inputs.timestampsSeconds.length, inputs.estimatedRobotPoses.length);
                assertEquals(inputs.timestampsSeconds.length, inputs.tagCounts.length);
                assertEquals(inputs.timestampsSeconds.length, inputs.averageTagDistancesMeters.length);
                assertEquals(inputs.timestampsSeconds.length, inputs.maxTagAmbiguities.length);
                for (int i = 0; i < inputs.timestampsSeconds.length; i++) {
                    assertEquals(received, inputs.timestampsSeconds[i]);
                    assertEquals(received, inputs.estimatedRobotPoses[i].getX());
                    assertEquals(1, inputs.tagCounts[i]);
                    assertEquals(received % 2 + 1, inputs.averageTagDistancesMeters[i]);
                    assertEquals(received % 2 == 0 ? 0.1 : 0.15, inputs.maxTagAmbiguities[i]);
                    received++;
                }
                if (finished) break;
                Thread.yield();
            }
            refresh.get(2, TimeUnit.SECONDS);
            assertEquals(2000, received);
        } finally {
            worker.shutdownNow();
            assertTrue(worker.awaitTermination(2, TimeUnit.SECONDS));
        }
    }
}
