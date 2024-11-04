package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import monologue.Logged
import monologue.Monologue


@Suppress("unused")
object UnitTesting {
    private val TICK_RATE: Time = 0.02.seconds
    private val globalCloseables: MutableList<AutoCloseable> = mutableListOf()

    /** Adds an AutoCloseable that is automatically closed at the end of all unit tests. */
    fun addGlobalCloseable(closeable: AutoCloseable) { globalCloseables.add(closeable) }

    /** Sets up DS and initializes HAL with default values and asserts that it doesn't fail.  */
    fun setup(disableMonologue: Boolean = true) {
        assert(HAL.initialize(500, 0)){ "HAL did not initialize." }
        DriverStationSim.setEnabled(true)
        DriverStationSim.setTest(true)
        DriverStationSim.notifyNewData()
        if (disableMonologue) {
            Monologue.setupMonologueDisabled(object: Logged {}, "", false)
            DataLogManager.stop()
        }
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
    fun fastForward(time: Time) {
        fastForward((time.inUnit(seconds) / TICK_RATE.inUnit(seconds)).toInt())
    }

    /**
     * Runs the command until it ends or the [timeout] is exceeded.
     */
    fun run(command: Command, timeout: Time) {
        CommandScheduler.getInstance().schedule(command)
        fastForward(1)
        fastForward(timeout)
    }
}