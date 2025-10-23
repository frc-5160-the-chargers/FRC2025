package frc.chargers.commands;

import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class NonBlockingCmds {
    public static Command sequence(Command... commands) {
        var allReqs = new HashSet<Subsystem>();
        for (var command: commands) {
            allReqs.addAll(command.getRequirements());
        }
        var group = new SequentialCommandGroup();
        group.addRequirements();
        for (var command: commands) {
            var requirementsToIdle = new HashSet<>(allReqs);
            requirementsToIdle.removeAll(command.getRequirements());
            group.addCommands(command.deadlineFor(new IdleAll(requirementsToIdle)));
        }
        return group.withName("NonBlockingSequentialCommandGroup");
    }

    public static Command parallel(Command... commands) {
        var group = new ParallelCommandGroup();
        var numEnded = new AtomicInteger();
        for (var cmd: commands) {
            group.addCommands(
                cmd
                    .finallyDo(numEnded::getAndIncrement)
                    .andThen(new IdleAll(cmd.getRequirements()))
            );
        }
        return group
            .until(() -> numEnded.get() == commands.length)
            .withName("NonBlockingParallelCommandGroup");
    }

    public static Command deadline(Command... commands) {
        var otherCmds = new Command[commands.length - 1];
        System.arraycopy(commands, 1, otherCmds, 1, commands.length - 1);
        return parallel(otherCmds)
            .withDeadline(commands[0])
            .withName("NonBlockingParallelDeadlineGroup");
    }

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
                    defaultCmds.add(defaultCmd);
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
