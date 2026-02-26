package frc.lib.drive;

import frc.robot.generated.Main2026TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import java.util.function.Supplier;

public enum DriveChassis {
    MAIN2026(Main2026TunerConstants.class, Main2026TunerConstants::createDrivetrain)

    ;

    private final Class<?> constantsClass;
    private final Supplier<CommandSwerveDrivetrain> createDrivetrainSupplier;

    DriveChassis(Class<?> constantsClass, Supplier<CommandSwerveDrivetrain> createDrivetrainSupplier) {
        this.constantsClass = constantsClass;
        this.createDrivetrainSupplier = createDrivetrainSupplier;
    }

    public CommandSwerveDrivetrain createDrivetrain() {
        return this.createDrivetrainSupplier.get();
    }

    public <T> T getField(String name) {
        try {
            return (T) constantsClass.getField(name).get(null);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("Failed to get field " + name + " from " + constantsClass.getName(), e);
        }
    }
}
