package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand

/**
 * A utility function that runs the [toRun] block
 * after a certain [Time] has elapsed.
 *
 * ```
 * waitThenRun(0.5.seconds) {
 *     println("This prints 0.5 seconds after.")
 * }
 */
fun waitThenRun(time: Time, toRun: () -> Unit) {
    CommandScheduler.getInstance().schedule(
        WaitCommand(time.inUnit(seconds))
            .andThen(InstantCommand(toRun))
            .ignoringDisable(true)
    )
}