package frc.robot;

/**
 * Holder for all CAN device IDs besides drivetrain devices
 */
public enum CanID {
    INTAKE_MOTOR(9),
    INTAKE_DEPLOY_MOTOR(10),
    INDEXER(10);

    private int deviceID;

    CanID(int deviceID) {
        this.deviceID = deviceID;
    }

    public int getID() {
        return deviceID;
    }
}

