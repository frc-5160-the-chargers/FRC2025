@file:Suppress("unused")
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.util.ErrorMessages
import edu.wpi.first.wpilibj2.command.*
import java.util.function.BooleanSupplier
import java.util.function.Supplier

/**
 * Namespace for command factory methods.
 * This class is a direct copy of wpilib's [Commands] namespace,
 * aside from 4 new factory methods below(that work better with kotlin lambda syntax).
 * Example:
 * ```
 * val old = Commands.runOnce({ println("Hello there!") }, drivetrain)
 * val old2 = Commands.sequence(command1, command2, command3)
 *
 * val new1 = Cmd.runOnce(drivetrain) { println("Nice!") }
 * val new2 = Cmd.sequence(command1, Cmd.parallel(...)) // identical to wpilib
 */
object Cmd {
    // new factory methods
    /**
     * Constructs a command that runs an action once and finishes.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see InstantCommand
     */
    fun runOnce(vararg requirements: Subsystem, action: () -> Unit): Command {
        return InstantCommand(action, *requirements)
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see RunCommand
     */
    fun run(vararg requirements: Subsystem, action: () -> Unit): Command {
        return RunCommand(action, *requirements)
    }

    /**
     * Constructs a command that does nothing, finishing after a specified duration.
     *
     * @param time after how long the command finishes; uses kmeasure time.
     * @return the command
     * @see WaitCommand
     */
    fun wait(time: com.batterystaple.kmeasure.quantities.Time): Command {
        return WaitCommand(time.inUnit(seconds))
    }

    /**
     * Constructs a command that runs the [commands] and the [deadlineCommand]
     * in parallel, stopping when the [deadlineCommand] finishes.
     * Alternative syntax to the [deadline] factory that forces named command usage
     * to declare the deadline; which increases clarity.
     *
     * @param commands
     * @param deadlineCommand
     * @see ParallelRaceGroup
     */
    fun parallel(vararg commands: Command, deadlineCommand: Command): Command {
        return ParallelDeadlineGroup(deadlineCommand, *commands)
    }


    // Copied from wpilib
    /**
     * Constructs a command that does nothing, finishing immediately.
     *
     * @return the command
     */
    fun none(): Command {
        return InstantCommand()
    }

    /**
     * Constructs a command that does nothing until interrupted.
     *
     * @param requirements Subsystems to require
     * @return the command
     */
    fun idle(vararg requirements: Subsystem): Command {
        return run({}, *requirements)
    }

    // Action Commands
    /**
     * Constructs a command that runs an action once and finishes.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see InstantCommand
     */
    fun runOnce(action: Runnable, vararg requirements: Subsystem): Command {
        return InstantCommand(action, *requirements)
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see RunCommand
     */
    fun run(action: Runnable, vararg requirements: Subsystem): Command {
        return RunCommand(action, *requirements)
    }

    /**
     * Constructs a command that runs an action once and another action when the command is
     * interrupted.
     *
     * @param start the action to run on start
     * @param end the action to run on interrupt
     * @param requirements subsystems the action requires
     * @return the command
     * @see StartEndCommand
     */
    fun startEnd(start: Runnable, end: Runnable, vararg requirements: Subsystem): Command {
        return StartEndCommand(start, end, *requirements)
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted, and then runs a
     * second action.
     *
     * @param run the action to run every iteration
     * @param end the action to run on interrupt
     * @param requirements subsystems the action requires
     * @return the command
     */
    fun runEnd(run: Runnable, end: Runnable, vararg requirements: Subsystem): Command {
        ErrorMessages.requireNonNullParam(end, "end", "Command.runEnd")
        return FunctionalCommand(
            {}, run,
            { interrupted: Boolean -> end.run() },
            { false }, *requirements
        )
    }

    /**
     * Constructs a command that runs an action once, and then runs an action every iteration until
     * interrupted.
     *
     * @param start the action to run on start
     * @param run the action to run every iteration
     * @param requirements subsystems the action requires
     * @return the command
     */
    fun startRun(start: Runnable, run: Runnable, vararg requirements: Subsystem): Command {
        return FunctionalCommand(
            start, run,
            { interrupted: Boolean -> },
            { false }, *requirements
        )
    }

    /**
     * Constructs a command that prints a message and finishes.
     *
     * @param message the message to print
     * @return the command
     * @see PrintCommand
     */
    fun print(message: String): Command {
        return PrintCommand(message)
    }

    // Idling Commands
    /**
     * Constructs a command that does nothing, finishing after a specified duration.
     *
     * @param seconds after how long the command finishes
     * @return the command
     * @see WaitCommand
     */
    fun waitSeconds(seconds: Double): Command {
        return WaitCommand(seconds)
    }

    /**
     * Constructs a command that does nothing, finishing once a condition becomes true.
     *
     * @param condition the condition
     * @return the command
     * @see WaitUntilCommand
     */
    fun waitUntil(condition: BooleanSupplier): Command {
        return WaitUntilCommand(condition)
    }

    // Selector Commands
    /**
     * Runs one of two commands, based on the boolean selector function.
     *
     * @param onTrue the command to run if the selector function returns true
     * @param onFalse the command to run if the selector function returns false
     * @param selector the selector function
     * @return the command
     * @see ConditionalCommand
     */
    fun either(onTrue: Command, onFalse: Command, selector: BooleanSupplier): Command {
        return ConditionalCommand(onTrue, onFalse, selector)
    }

    /**
     * Runs one of several commands, based on the selector function.
     *
     * @param <K> The type of key used to select the command
     * @param selector the selector function
     * @param commands map of commands to select from
     * @return the command
     * @see SelectCommand
    </K> */
    fun <K> select(commands: Map<K, Command>, selector: Supplier<K>): Command {
        return SelectCommand(commands, selector)
    }

    /**
     * Runs the command supplied by the supplier.
     *
     * @param supplier the command supplier
     * @param requirements the set of requirements for this command
     * @return the command
     * @see DeferredCommand
     */
    fun defer(supplier: Supplier<Command>, requirements: Set<Subsystem>): Command {
        return DeferredCommand(supplier, requirements)
    }

    /**
     * Constructs a command that schedules the command returned from the supplier when initialized,
     * and ends when it is no longer scheduled. The supplier is called when the command is
     * initialized.
     *
     * @param supplier the command supplier
     * @return the command
     * @see ProxyCommand
     */
    @Deprecated(
        """The ProxyCommand supplier constructor has been deprecated in favor of directly
        proxying a {@link DeferredCommand}, see ProxyCommand documentation for more details. As a
        replacement, consider using `defer(supplier).asProxy()`.
    """
    )
    fun deferredProxy(supplier: Supplier<Command>): Command {
        return ProxyCommand(supplier)
    }

    // Command Groups
    /**
     * Runs a group of commands in series, one after the other.
     *
     * @param commands the commands to include
     * @return the command group
     * @see SequentialCommandGroup
     */
    fun sequence(vararg commands: Command): Command {
        return SequentialCommandGroup(*commands)
    }

    /**
     * Runs a group of commands in series, one after the other. Once the last command ends, the group
     * is restarted.
     *
     * @param commands the commands to include
     * @return the command group
     * @see SequentialCommandGroup
     *
     * @see Command.repeatedly
     */
    fun repeatingSequence(vararg commands: Command): Command {
        return sequence(*commands).repeatedly()
    }

    /**
     * Runs a group of commands at the same time. Ends once all commands in the group finish.
     *
     * @param commands the commands to include
     * @return the command
     * @see ParallelCommandGroup
     */
    fun parallel(vararg commands: Command): Command {
        return ParallelCommandGroup(*commands)
    }

    /**
     * Runs a group of commands at the same time. Ends once any command in the group finishes, and
     * cancels the others.
     *
     * @param commands the commands to include
     * @return the command group
     * @see ParallelRaceGroup
     */
    fun race(vararg commands: Command): Command {
        return ParallelRaceGroup(*commands)
    }

    /**
     * Runs a group of commands at the same time. Ends once a specific command finishes, and cancels
     * the others.
     *
     * @param deadline the deadline command
     * @param otherCommands the other commands to include
     * @return the command group
     * @see ParallelDeadlineGroup
     *
     * @throws IllegalArgumentException if the deadline command is also in the otherCommands argument
     */
    fun deadline(deadline: Command, vararg otherCommands: Command): Command {
        return ParallelDeadlineGroup(deadline, *otherCommands)
    }
}