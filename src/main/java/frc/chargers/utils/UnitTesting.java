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

    /** Sets up DS and initializes HAL with default values and asserts that it doesn't fail. */
    public static void setupTests() {
        assert HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
    }

    /**
     * Resets CommandScheduler and closes all subsystems. Please call in an @AfterEach method!
     *
     * @param autoCloseables All subsystems/objects that need to be closed
     */
    public static void reset(AutoCloseable... autoCloseables) throws Exception {
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().cancelAll();
        for (AutoCloseable closeable : autoCloseables) {
            closeable.close();
        }
    }

    /**
     * Runs CommandScheduler and updates timer repeatedly to fast forward subsystems and run commands.
     *
     * @param ticks The number of times CommandScheduler is run
     */
    public static void fastForward(int ticks) {
        for (int i = 0; i < ticks; i++) {
            CommandScheduler.getInstance().run();
            SimHooks.stepTiming(TICK_RATE.in(Seconds));
        }
    }

    /**
     * Fasts forward in time by running CommandScheduler and updating timer
     */
    public static void fastForward(Time time) {
        fastForward((int) (time.in(Seconds) / TICK_RATE.in(Seconds)));
    }

    /**
     * Schedules and runs a command
     *
     * @param command The command to run.
     */
    public static void run(Command command, double seconds) {
        command.schedule();
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules and runs a command.
     *
     * @param command The command to run.
     * @param runs The number of times CommandScheduler is run.
     */
    public static void run(Command command, int runs) {
        command.schedule();
        fastForward(runs);
    }

    /**
     * Schedules a command and runs it until it ends. Be careful -- if the command you give never
     * ends, this will be an infinate loop!
     *
     * @param command
     */
    public static void runToCompletion(Command command) {
        command.schedule();
        fastForward(1);
        while (command.isScheduled()) {
            fastForward(1);
        }
    }
}