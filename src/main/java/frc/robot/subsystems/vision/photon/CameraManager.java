package frc.robot.subsystems.vision.photon;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

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
                "back",
                 new Transform3d(
                    Inches.of(-13.6), // checked
                    Inches.of(-2.5), //checked
                    Inches.of(8.875), //checked
                    new Rotation3d(
                        Degrees.of(0),
                        Degrees.of(-65),
                        Degrees.of(180)
                    )
                 )
            )
        );

        registerCamera(
            new Camera(
                "right",
                new Transform3d(
                    Inches.of(-1.5), //checked
                    Inches.of(-13.55), //checked
                    Inches.of(8.9), //checked
                    new Rotation3d(
                        Degrees.of(0),
                        Degrees.of(-65), // negative pitch = tilted upward
                        Degrees.of(-90)
                    )
                )
            )
        );

        registerCamera(
            new Camera(
                "left",
                 new Transform3d(
                    Inches.of(-2.875), // checked
                    Inches.of(-13.5), // checked 
                    Inches.of(8.8), // checked
                 new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-65), // negative pitch = tilted upward
                    Degrees.of(90))
                )
            )
        );
    }
}
