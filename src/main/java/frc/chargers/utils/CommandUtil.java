package frc.chargers.utils;

import edu.wpi.first.wpilibj2.command.*;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

public class CommandUtil {
    public static Command nonBlockingSequence(Command... commands) {
        var allReqs = new HashSet<Subsystem>();
        for (var command: commands) {
            allReqs.addAll(command.getRequirements());
        }
        var group = new SequentialCommandGroup();
        for (var command: commands) {
            var idleOtherSubsystems = new ParallelCommandGroup();
            for (var req: allReqs) {
                if (command.getRequirements().contains(req)) continue;
                idleOtherSubsystems.addCommands(
                    Commands.defer(req::getDefaultCommand, Set.of(req))
                );
            }
            group.addCommands(command.deadlineFor(idleOtherSubsystems));
        }
        return group.withName("NonBlockingSequentialCommandGroup");
    }

    public static Command nonBlockingParallel(Command... commands) {
        var group = new ParallelCommandGroup();
        var numEnded = new AtomicInteger();
        for (var cmd: commands) {
            var idleAtEnd = new ParallelCommandGroup();
            for (var req: cmd.getRequirements()) {
                idleAtEnd.addCommands(
                    Commands.defer(req::getDefaultCommand, Set.of(req))
                );
            }
            group.addCommands(
                cmd
                    .finallyDo(numEnded::getAndIncrement)
                    .andThen(idleAtEnd)
            );
        }
        return group
            .until(() -> numEnded.get() == commands.length)
            .withName("NonBlockingParallelCommandGroup");
    }

    public static Command nonBlockingDeadline(Command... commands) {
        var otherCmds = new Command[commands.length - 1];
        System.arraycopy(commands, 1, otherCmds, 1, commands.length - 1);
        return nonBlockingParallel(otherCmds)
            .withDeadline(commands[0])
            .withName("NonBlockingParallelDeadlineGroup");
    }
}
