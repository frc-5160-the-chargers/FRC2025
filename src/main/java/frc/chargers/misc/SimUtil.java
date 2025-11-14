package frc.chargers.misc;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.units.Units.*;

public class SimUtil {
    private SimUtil() {}

    /**
     * Applies a current limit to an input voltage
     * to a physics sim.
     * @return the current-limited voltage
     */
    public static double currentLimitVoltage(
        double inputVoltage,
        DCMotor gearbox,
        Current currentLimit,
        AngularVelocity mechanismVel,
        double reduction
    ) {
        double motorCurrentVelocityRadPerSec = mechanismVel.in(RadiansPerSecond) * reduction;
        double currentLimitAmps = currentLimit.in(Amps);
        double currentAtRequestedVoltageAmps = gearbox.getCurrent(motorCurrentVelocityRadPerSec, inputVoltage);
        double limitedVoltage = inputVoltage;
        boolean currentTooHigh = Math.abs(currentAtRequestedVoltageAmps) > 1.2 * currentLimitAmps;
        if (currentTooHigh) {
            double limitedCurrent = Math.copySign(currentLimitAmps, currentAtRequestedVoltageAmps);
            limitedVoltage = gearbox.getVoltage(gearbox.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec);
        }
        if (Math.abs(limitedVoltage) > Math.abs(inputVoltage)) {
            limitedVoltage = inputVoltage;
        }
        return limitedVoltage;
    }

    public static double ampsToVolts(
        double requestAmps,
        DCMotor gearbox,
        AngularVelocity mechanismVel
    ) {
        double torqueNM = gearbox.getTorque(requestAmps);
        return gearbox.getVoltage(torqueNM, mechanismVel.in(RadiansPerSecond));
    }

    public static double[] simulateOdoTimestamps() {
        final double[] values = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < values.length; i++) {
            values[i] = Timer.getTimestamp()
                - 0.02
                + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return values;
    }
}
