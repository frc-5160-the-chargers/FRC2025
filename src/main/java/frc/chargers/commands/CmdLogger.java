package frc.chargers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.chargers.misc.Tracer;
import org.littletonrobotics.junction.Logger;

import java.util.HashSet;
import java.util.Set;

/**
 * A utility class for logging commands - inspired by FRC 1683.
 * The "Flowchart" field should be dragged into the "discrete fields" section on AdvantageScope's line graph,
 * while the "InterruptMsgs" and "DuplicateNameMsgs" fields should be dragged into the console section
 * (click +, then "Console", then drag the field in).
 * Green commands are default commands, while yellow commands are non-default ones.
 */
public class CmdLogger {
    private static boolean hasStarted = false, logDupeNames = false;
    private static final Set<String>
        runningCmdNames = new HashSet<>(),
        runningDefaultCmdNames = new HashSet<>(),
        ranCmdNames = new HashSet<>(),
        interruptMsgs = new HashSet<>(),
        duplicateMsgs = new HashSet<>();
    private static char blank = ' ';

    /**
     * Runs the logger. This must be run periodically, before CommandScheduler.getInstance.run().
     * Note that commands that finish instantly don't appear in the flowchart;
     * rather, see the RanAtLeastOnce field.
     */
    public static void periodic(boolean logDuplicateNames) {
        Tracer.startTrace("Command Logger");
        if (!hasStarted) {
            start(logDuplicateNames);
            hasStarted = true;
        }
        Logger.recordOutput("Commands/Flowchart/warnings", runningCmdNames.toArray(new String[0]));
        Logger.recordOutput("Commands/Flowchart/infos", runningDefaultCmdNames.toArray(new String[0]));
        Logger.recordOutput("Commands/FinishedAtLeastOnce", ranCmdNames.toArray(new String[0]));
        if (!interruptMsgs.isEmpty()) {
            var msg = String.join("\n", interruptMsgs) + blank;
            interruptMsgs.clear();
            blank = blank == ' ' ? '\t' : ' '; // ensures that duplicate msgs are logged
            Logger.recordOutput("Commands/InterruptMsgs", msg);
        }
        if (!duplicateMsgs.isEmpty()) {
            Logger.recordOutput("Commands/DuplicateNameMsgs", String.join("\n", duplicateMsgs));
            duplicateMsgs.clear();
        }
        Tracer.endTrace();
    }

    /**
     * Creates a new command that will be logged inside a command group.
     * This is not necessary if you're using one of the NonBlockingCmds.
     */
    public static Command logNestedCmd(Command cmd) {
        return new WrapperCommand(cmd) {
            @Override
            public void initialize() {
                register(cmd);
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                unregister(cmd);
            }
        };
    }

    private static void start(boolean logDuplicateNames) {
        Logger.recordOutput("Commands/Flowchart/.type", "Alerts");
        Logger.recordOutput("Commands/InterruptMsgs", "");
        if (logDuplicateNames) {
            logDupeNames = true;
            Logger.recordOutput(
                "Commands/DuplicateNameMsgs",
                "The following commands were scheduled when another command of the same name was already active:"
            );
        }
        var scd = CommandScheduler.getInstance();
        scd.onCommandInitialize(CmdLogger::register);
        scd.onCommandFinish(CmdLogger::unregister);
        scd.onCommandInterrupt((cmd, interrupter) -> {
            unregister(cmd);
            interrupter.ifPresent(it -> interruptMsgs.add(it.getName() + " interrupted " + cmd.getName()));
        });
    }

    private static void register(Command cmd) {
        for (var s: cmd.getRequirements()) {
            if (s.getDefaultCommand() == cmd) {
                runningDefaultCmdNames.add(cmd.getName());
                return;
            }
        }
        boolean notDupe = runningCmdNames.add(cmd.getName());
        if (!notDupe && logDupeNames) duplicateMsgs.add(cmd.getName());
    }

    private static void unregister(Command cmd) {
        ranCmdNames.add(cmd.getName());
        runningCmdNames.remove(cmd.getName());
        runningDefaultCmdNames.remove(cmd.getName());
    }
}
