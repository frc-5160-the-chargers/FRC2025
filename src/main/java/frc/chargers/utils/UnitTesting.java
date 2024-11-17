package frc.chargers.utils;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class UnitTesting {
    public static final Time TICK_RATE = Seconds.of(0.02);
    private static boolean isUnitTest = false;

    /** Sets up DS and initializes HAL with default values and asserts that it doesn't fail. */
    public static void setupTest() {
        assert HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        isUnitTest = true;
    }

    /**
     * Resets CommandScheduler and closes all subsystems. Please call in an @AfterEach method!
     *
     * @param autoCloseables All subsystems/objects that need to be closed
     */
    public static void resetTest(AutoCloseable... autoCloseables) throws Exception {
        if (!isUnitTest) return;
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().cancelAll();
        for (AutoCloseable closeable : autoCloseables) {
            closeable.close();
        }
    }

    /**
     * Runs CommandScheduler and updates timer repeatedly
     * to fast-forward subsystems and run commands.
     *
     * @param ticks The number of times CommandScheduler is run
     */
    public static void runTicks(int ticks) {
        if (!isUnitTest) return;
        for (int i = 0; i < ticks; i++) {
            CommandScheduler.getInstance().run();
            SimHooks.stepTiming(TICK_RATE.in(Seconds));
        }
    }

    /**
     * Fasts forward in time by running CommandScheduler and updating timer.
     */
    public static void fastForward(Time time) {
        runTicks((int) (time.in(Seconds) / TICK_RATE.in(Seconds)));
    }
    
    /**
     * Schedules and runs a command.
     */
    public static void runUntilComplete(Command command, Time timeout) {
        if (!isUnitTest) return;
        CommandScheduler.getInstance().schedule(command);
        fastForward(timeout);
    }
    
    /**
     * Schedules a command without running it.
     * To run the command for a certain number of ticks, use <code>UnitTesting.runTicks(1)</code>
     */
    public static void schedule(Command command) {
        if (!isUnitTest) return;
        CommandScheduler.getInstance().schedule(command);
    }
}