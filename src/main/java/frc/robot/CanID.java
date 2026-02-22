package frc.robot;

/**
 * Holder for all CAN device IDs besides drivetrain devices
 */
public enum CanID {
    INTAKE_MOTOR(9),
    INTAKE_DEPLOY_MOTOR(11),
    INDEXER(10);

    private final int deviceID;

    CanID(int deviceID) {
        this.deviceID = deviceID;
    }

    public int getID() {
        return deviceID;
    }
}

