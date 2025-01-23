package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.chargers.field.ScoringLevel;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import lombok.RequiredArgsConstructor;


@RequiredArgsConstructor
public class AutoCommands {
	private final RobotCommands botCommands;
	private final AutoFactory autoFactory;
	private final CoralIntake coralIntake;
	private final CoralIntakePivot coralIntakePivot;
	private final Elevator elevator;
	
	/**
	 * A shorter way to declare <code>step1.done().onTrue(step2.cmd())</code>
	 */
	private static void chain(AutoTrajectory step1, AutoTrajectory step2) {
		step1.done().onTrue(step2.cmd());
	}
	
	/**
	 * A shorter way to run <code>step1.done().onTrue(step2.andThen(step3.cmd()));</code>
	 * <br />
	 * A ScheduleCommand is used to prevent requirement conflicts and reduce bugs.
	 */
	private static void chain(AutoTrajectory step1, Command step2, AutoTrajectory step3) {
		step1.done().onTrue(
			step2.andThen(new ScheduleCommand(step3.cmd()))
		);
	}
	
	// TODO make this a 3 piece instead
	public Command multiPieceCenter() {
		var routine = autoFactory.newRoutine("TwoPiece");
		var lineToReef1 = routine.trajectory("lineToReef1");
		var reef1ToSource = routine.trajectory("reef1ToSource");
		var sourceToReef3 = routine.trajectory("sourceToReef3");
		
		routine.active().onTrue(
			lineToReef1.resetOdometry().andThen(lineToReef1.cmd())
		);
		
		// autoScore will automatically stow one gamepiece is out
		lineToReef1.atTime(0.83).onTrue(botCommands.autoScore(new ScoringLevel(4)));
		// wait until elevator has retracted enough so that CoG is low, then drive next path
		chain(lineToReef1, Commands.waitUntil(() -> elevator.extensionHeight() < 0.2), reef1ToSource);
		
		reef1ToSource.atTime(1.27).onTrue(botCommands.sourceIntake());
		chain(reef1ToSource, Commands.waitUntil(coralIntake.hasCoral), sourceToReef3);
		
		sourceToReef3.active().whileTrue(botCommands.stow());
		sourceToReef3.atTime(1.9).onTrue(botCommands.autoScore(new ScoringLevel(4)));
		
		return routine.cmd();
	}
	
}
