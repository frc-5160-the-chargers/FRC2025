package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoCommands {
	private final RobotCommands botCommands;
	private final AutoFactory autoFactory;
	private final CoralIntake coralIntake;
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
			step2.andThen(Commands.runOnce(() -> step3.cmd().schedule()))
		);
	}
	
	public Command pathTest() {
		return autoFactory.resetOdometry("SimplePath").andThen(
			Commands.runOnce(() -> System.out.println("SKDJFLJKLJSLKDJFKLSJDKFJKSDLFKLSDJFKLSDJ")),
			autoFactory.trajectoryCmd("SimplePath")
		).withName("pathTest");
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
		
		// at time of 0.83 secs, start to move to scoring position
		lineToReef1.atTime(0.83).onTrue(botCommands.scoreSequence(4));
		// wait until elevator has retracted enough so that CoG is low, then drive next path
		chain(lineToReef1, Commands.waitUntil(elevator.readyForMovement), reef1ToSource);
		
		reef1ToSource.atTime(1.27).onTrue(botCommands.sourceIntake());
		chain(reef1ToSource, Commands.waitUntil(coralIntake.hasCoral), sourceToReef3);
		
		sourceToReef3.active().whileTrue(botCommands.moveTo(Setpoint.STOW));
		sourceToReef3.atTime(1.8).onTrue(botCommands.scoreSequence(4));
		
		return routine.cmd();
	}

	/*
	autoFactory.trajectoryCmd("pathName").andThen(
		command1,
		command2,
		command3
	)
	 */
	public Command figureEight() {
		var routine = autoFactory.newRoutine("FigureEight");
		var figureEight = routine.trajectory("FigureEight");

		routine.active().onTrue(
				figureEight.resetOdometry().andThen(figureEight.cmd())
		);

		return routine.cmd();
	}
}
