package frc.robot;

/**
 * Holder for all CAN device IDs besides drivetrain devices
 */
public enum CanID {
    INTAKE(9)
    ;

    private int deviceID;

    CanID(int deviceID) {
        this.deviceID = deviceID;
    }

    public int getID() {
        return deviceID;
    }
}

