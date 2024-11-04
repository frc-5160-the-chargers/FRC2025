package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler


@Suppress("unused")
object UnitTesting {
    private val TICK_RATE: Time = 0.02.seconds
    private val globalCloseables: MutableList<AutoCloseable> = mutableListOf()

    /** Adds an AutoCloseable that is automatically closed at the end of all unit tests. */
    fun addGlobalCloseable(closeable: AutoCloseable) { globalCloseables.add(closeable) }

    /** Sets up DS and initializes HAL with default values and asserts that it doesn't fail.  */
    fun setup() {
        assert(HAL.initialize(500, 0))
        DriverStationSim.setEnabled(true)
        DriverStationSim.setTest(true)
        DriverStationSim.notifyNewData()
    }

    /**
     * Resets CommandScheduler and closes all subsystems. Please call in an @AfterEach method!
     */
    fun cleanup() {
        CommandScheduler.getInstance().unregisterAllSubsystems()
        CommandScheduler.getInstance().cancelAll()
        globalCloseables.forEach { it.close() }
    }

    /**
     * Runs CommandScheduler and updates timer repeatedly to fast forward subsystems and run commands.
     *
     * @param ticks The number of times CommandScheduler is run
     */
    fun fastForward(ticks: Int) {
        for (i in 0 until ticks) {
            CommandScheduler.getInstance().run()
            SimHooks.stepTiming(TICK_RATE.inUnit(seconds))
        }
    }

    /**
     * Runs CommandScheduler and updates timer to fast forward subsystems by 4 seconds and run
     * commands.
     */
    fun fastForward(time: Time = 4.seconds) {
        fastForward((time.inUnit(seconds) / TICK_RATE.inUnit(seconds)).toInt())
    }

    /**
     * Schedules and runs a command
     *
     * @param command The command to run.
     */
    fun run(command: Command) {
        command.schedule()
        CommandScheduler.getInstance().run()
    }

    /**
     * Schedules a command and runs it until it ends. Be careful -- if the command you give never
     * ends, this will be an infinite loop!
     *
     * @param command
     */
    fun runToCompletion(command: Command) {
        command.schedule()
        fastForward(1)
        var totalTicks = 0
        while (command.isScheduled) {
            fastForward(1)
            if (++totalTicks > 1000) error("Command has been executing for over 20 seconds; make sure to put a stop condition.")
        }
    }
}