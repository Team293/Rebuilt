package frc.robot;

/**
 * Holder for all CAN device IDs besides drivetrain devices
 */
public enum CanID {
    INDEXER_MOTOR(10),

    FLYWHEEL_MOTOR(20),
    HOOD_MOTOR(12),
    HOOD_ENCODER(14),

    TURRET_MOTOR(13),

    INTAKE_MOTOR(9),
    INTAKE_DEPLOY_MOTOR(11),

    FINDEXER_MOTOR(15)

    ;

    private final int deviceID;

    CanID(int deviceID) {
        this.deviceID = deviceID;
    }

    public int getID() {
        return deviceID;
    }
}

