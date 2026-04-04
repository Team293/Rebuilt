package frc.robot;

/**
 * Holder for all CAN device IDs besides drivetrain devices
 */
public enum CanID {
    TRIGGER_MOTOR(55),

    FLYWHEEL_MOTOR(20),
    HOOD_MOTOR(50),
    HOOD_ENCODER(31),

    TURRET_MOTOR(0),

    INTAKE_MOTOR(41),
    INTAKE_DEPLOY_MOTOR(11),

    TURRET_PINION_CANCODER(20), // TODO: ???????????????????
    TURRET_FOLLOWER_CANCODER(19),

    FINDEXER_MOTOR(18);


    private final int deviceID;

    CanID(int deviceID) {
        this.deviceID = deviceID;
    }

    public int getID() {
        return deviceID;
    }
}

