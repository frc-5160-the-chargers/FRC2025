package testingutil;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.utils.data.StatusSignalRefresher;
import org.ironmaple.simulation.SimulatedArena;
import org.jetbrains.annotations.Nullable;
import org.junit.jupiter.api.BeforeEach;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;

public abstract class StandardUnitTest {
	@BeforeEach
	void requiredSetup() {
		assertTrue(HAL.initialize(500, 0));
		DriverStationSim.setEnabled(true);
		DriverStationSim.setTest(true);
		DriverStationSim.notifyNewData();
	}
	
	public static final Time TICK_RATE = Seconds.of(0.02);
	
	public static void fastForward(@Nullable CommandScheduler scheduler, Time time) {
		fastForward(scheduler, timeToTicks(time));
	}
	
	/**
	 * Runs CommandScheduler and updates timer repeatedly
	 * to fast-forward subsystems and run commands.
	 *
	 * @param ticks The number of times CommandScheduler is run
	 */
	public static void fastForward(@Nullable CommandScheduler scheduler, int ticks) {
		for (int i = 0; i < ticks; i++) {
			HAL.simPeriodicBefore();
			StatusSignalRefresher.periodic();
			Timer.delay(0.02);
			if (scheduler != null) scheduler.run();
			SimulatedArena.getInstance().simulationPeriodic();
			HAL.simPeriodicAfter();
		}
	}
	
	/**
	 * Fetches the number of ticks that occur in the time interval.
	 */
	public static int timeToTicks(Time time) {
		return (int) (time.in(Seconds) / TICK_RATE.in(Seconds));
	}
}
