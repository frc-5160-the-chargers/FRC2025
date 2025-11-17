package frc.robot.components.gyro;

import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class GyroSim extends Gyro {
    private final GyroSimulation gyroSimulation;

    public GyroSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void refreshData(GyroDataAutoLogged inputs) {
        inputs.connected = true;
        inputs.yaw = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.cachedYawValues = gyroSimulation.getCachedGyroReadings();
        inputs.cachedTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < inputs.cachedTimestamps.length; i++) {
            inputs.cachedTimestamps[i] = Timer.getTimestamp()
                - 0.02
                + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
    }
}
