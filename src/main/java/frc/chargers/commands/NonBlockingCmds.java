package frc.chargers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class NonBlockingCmds {
    /**
     * A command that runs the given commands one after another,
     * while running the default commands of non-active subsystems.
     */
    public static Command sequence(Command... commands) {
        var allReqs = new HashSet<Subsystem>();
        for (var command: commands) {
            allReqs.addAll(command.getRequirements());
        }
        var group = new SequentialCommandGroup();
        for (var command: commands) {
            var requirementsToIdle = new HashSet<>(allReqs);
            requirementsToIdle.removeAll(command.getRequirements());
            group.addCommands(
                CmdLogger.logNestedCmd(command)
                    .deadlineFor(new IdleAll(requirementsToIdle))
            );
        }
        return group.withName("NonBlockingSequence");
    }

    /**
     * A command that runs the given commands in parallel,
     * finishing when all commands finish,
     * while running the default commands of non-active subsystems. <br />
     * Deadline groups should be made with <code>NonBlockingCmds.parallel(a,b).withDeadline(c)</code>,
     * while race groups can just be made with <code>Commands.race()</code>.
     */
    public static Command parallel(Command... commands) {
        var group = new ParallelCommandGroup();
        var numEnded = new AtomicInteger();
        for (var command: commands) {
            group.addCommands(
                CmdLogger.logNestedCmd(command)
                    .finallyDo(numEnded::getAndIncrement)
                    .andThen(new IdleAll(command.getRequirements()))
            );
        }
        return group
            .until(() -> numEnded.get() == commands.length)
            .withName("NonBlockingParallelGroup");
    }

    /** This is a utility class. */
    private NonBlockingCmds() {}

    private static class IdleAll extends Command {
        private final Collection<Subsystem> subsystems;
        private final List<Command> defaultCmds = new ArrayList<>();

        public IdleAll(Collection<Subsystem> subsystems) {
            this.subsystems = subsystems;
        }

        @Override
        public void initialize() {
            for (var s: subsystems) {
                var defaultCmd = s.getDefaultCommand();
                if (defaultCmd != null) {
                    defaultCmds.add(CmdLogger.logNestedCmd(defaultCmd));
                    defaultCmd.initialize();
                }
            }
        }

        @Override
        public void execute() {
            for (var cmd: defaultCmds) {
                cmd.execute();
            }
        }
    }
}
