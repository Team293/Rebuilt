package frc.robot.subsystems.vision.photon;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

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
        registerCamera(
            new Camera(
                "left", 
                new Transform3d(
                    Inches.of(0),
                    Inches.of(-29.5/2),
                    Inches.of(12.9375),
                    new Rotation3d(
                        Degrees.of(90),
                        Degrees.of(0),
                        Degrees.of(0)
                    )
                )   
            )
        );

        registerCamera(
            new Camera(
                "right",
                 new Transform3d(
                    Inches.of(0),
                    Inches.of(-29.5/2),
                    Inches.of(9.25),
                    new Rotation3d(
                        Degrees.of(-90),
                        Degrees.of(0),
                        Degrees.of(0)
                    )
                 )
            )
        );
    }
}
