package frc.robot.components.gyro;

import edu.wpi.first.math.util.Units;
import frc.chargers.misc.SimUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroSim extends Gyro {
    private final GyroSimulation gyroSimulation;

    public GyroSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void refreshData(GyroDataAutoLogged inputs) {
        inputs.connected = true;
        inputs.yaw = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odoYawTimestamps = SimUtil.simulateOdoTimestamps();
        inputs.odoYawValues = gyroSimulation.getCachedGyroReadings();
    }
}
