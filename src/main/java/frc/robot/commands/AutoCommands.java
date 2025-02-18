package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
	 * A shorter way to run <code>step1.done().onTrue(step2.andThen(step3.cmd()));</code> <br />
	 */
	private static void chain(AutoTrajectory step1, Command step2, AutoTrajectory step3) {
		step1.done().onTrue(step2.andThen(step3.spawnCmd()));
	}
	
	public Command pathTest() {
		return autoFactory.resetOdometry("SimplePath").andThen(
			Commands.runOnce(() -> System.out.println("SKDJFLJKLJSLKDJFKLSJDKFJKSDLFKLSDJFKLSDJ")),
			autoFactory.trajectoryCmd("SimplePath")
		).withName("pathTest");
	}
	
	public Command multiPieceCenter() {
		var routine = autoFactory.newRoutine("ThreePiece");
		var lineToReef1 = routine.trajectory("lineToReef1");
		var reef1ToSource = routine.trajectory("reef1ToSource");
		var sourceToReef11 = routine.trajectory("sourceToReef11");
		var reef11ToSource = routine.trajectory("reef11ToSource");
		var sourceToReef10 = routine.trajectory("sourceToReef10");
		
		routine.active().onTrue(
			lineToReef1.resetOdometry().andThen(lineToReef1.spawnCmd())
		);
		
		lineToReef1.active().onTrue(
			botCommands.scoreSequence(4, lineToReef1.done())
		);
		// wait until elevator has retracted enough so that CoG is low, then drive next path
		chain(lineToReef1, Commands.waitUntil(elevator.readyForMovement), reef1ToSource);
		
		reef1ToSource.atTime(1.0).onTrue(botCommands.sourceIntake());
		chain(reef1ToSource, Commands.waitUntil(coralIntake.hasCoral), sourceToReef11);
		
		sourceToReef11.atTime(1.1).onTrue(botCommands.scoreSequence(4));
		chain(sourceToReef11, Commands.waitUntil(elevator.readyForMovement), reef11ToSource);
		
		reef11ToSource.atTime(0.5).onTrue(botCommands.sourceIntake());
		chain(reef11ToSource, Commands.waitUntil(coralIntake.hasCoral), sourceToReef10);
		
		sourceToReef10.atTime(1.1).onTrue(botCommands.scoreSequence(4));
		sourceToReef10.done().onTrue(
			Commands.print("DONEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
		);
		
		return routine.cmd();
	}
	
	public Command multiPieceTest() {
		var routine = autoFactory.newRoutine("MultiPieceTest");
		var lineToReef1 = routine.trajectory("lineToReef1");
		var reef1ToSource = routine.trajectory("reef1ToSource");
		var sourceToReef11 = routine.trajectory("sourceToReef11");
		var reef11ToSource = routine.trajectory("reef11ToSource");
		var sourceToReef10 = routine.trajectory("sourceToReef10");
		
		routine.active().onTrue(
			lineToReef1.resetOdometry().andThen(
				lineToReef1.cmd(),
				botCommands.scoreSequence(4),
				reef1ToSource.cmd(),
				sourceToReef11.cmd(),
				reef11ToSource.cmd(),
				sourceToReef10.cmd()
			)
		);
		return routine.cmd();
	}
	
	
	public Command figureEight() {
		var routine = autoFactory.newRoutine("FigureEight");
		var figureEight = routine.trajectory("FigureEight");

		routine.active().onTrue(
			figureEight.resetOdometry().andThen(figureEight.spawnCmd())
		);

		return routine.cmd();
	}
}
