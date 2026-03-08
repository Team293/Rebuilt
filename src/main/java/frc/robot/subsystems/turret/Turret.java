package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import frc.lib.subsystem.SpikeSystem;
import frc.robot.RobotContainer;

import frc.robot.subsystems.targeting.Targeting;
import frc.robot.subsystems.turret.calc.ShotCompensation;

public class Turret extends SpikeSystem<TurretIO.TurretIOInputs> {
    private static final double TURRET_ANGLE_THRESHOLD_DEG = 3.0; // degrees within target angle to be considered "at target"
    
    private TurretIO turretIO;
    private double targetAngleDeg = 0.0;

    public Turret() {
        super("Turret", new TurretIO.TurretIOInputs());
        System.out.println("Turret subsystem initialized");
    }
    
    /**
     *  Continuously sets the turret angle to point at the target position
     */ 
    @Override
    public void onPeriodic() {
        Logger.recordOutput("Turret/Degrees", io.turretAngleDegrees);

        // compensate for robot movement
        ShotCompensation.AdjustedShot shotData = Targeting.getShotData();

        if (shotData != null) {
            double newTargetAngleDeg = shotData.turretAngleDeg();
            this.targetAngleDeg = newTargetAngleDeg;

            this.turretIO.setTurretAngle(newTargetAngleDeg);
        }
    }
    
    /**
     * Sets up the data refresher for 
     */
    @Override
    protected Runnable setupDataRefresher() {
        this.turretIO = new TurretIOTalonFX(RobotContainer.getDrive());
        return useAsyncDataRefresher(this.turretIO);
    }

    /**
     * Determines if turret is at target angle within error bounds 
     * @return if turret is at angle within bounds 
     */
    public boolean isAtTargetAngle() {
        double angleError = Math.abs(super.io.turretAngleDegrees - targetAngleDeg);
        return angleError < TURRET_ANGLE_THRESHOLD_DEG;
    }
}
