package frc.robot.subsystems.vision.photon;

import java.util.ArrayList;
import java.util.List;

public class CameraManager {
    private static final List<Camera> cameras = new ArrayList<>();

    public static void registerCamera(Camera camera) {
        cameras.add(camera);
    }

    public static List<Camera> getCameras() {
        return cameras;
    }

    static {
        // register cameras

    }
}
