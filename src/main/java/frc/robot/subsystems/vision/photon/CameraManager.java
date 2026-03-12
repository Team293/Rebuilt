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
        // registerCamera(
        //     new Camera(
        //         "left", 
        //         new Transform3d(
        //             Inches.of(-13.8),
        //             Inches.of(7.5),
        //             Inches.of(22.3/4),
        //             new Rotation3d(
        //                 Degrees.of(0),
        //                 Degrees.of(60),
        //                 Degrees.of(90)
        //             )
        //         )   
        //     )
        // );

        // registerCamera(
        //     new Camera(
        //         "right",
        //          new Transform3d(
        //             Inches.of(13.8),
        //             Inches.of(5),
        //             Inches.of(6.3/4),
        //             new Rotation3d(
        //                 Degrees.of(0),
        //                 Degrees.of(60),
        //                 Degrees.of(-90)
        //             )
        //          )
        //     )
        // );

        registerCamera(
            new Camera(
                "left",
                new Transform3d(
                    Inches.of(-29.5/2),
                    Inches.of(0),
                    Inches.of(6.75),
                    new Rotation3d(
                        Degrees.of(0),
                        Degrees.of(-60), // negative pitch = tilted upward
                        Degrees.of(180)
                    )
                )
            )
        );
    }
}
