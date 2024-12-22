package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoCommands {
	private final AutoFactory autoFactory;
	
	private boolean hasNote() {
		return true; // dummy
	}
	
	public Command fourPiece() {
		var routine = autoFactory.newRoutine("FourPiece");
		var ampToM1 = routine.trajectory("AmpToM1");
		var m1ToC1 = routine.trajectory("M1ToC1");
		var c1ToScore = routine.trajectory("C1ToScore");
		var scoreToC2 = routine.trajectory("ScoreToC2");
		var c2ToScore = routine.trajectory("C2ToScore");
		var c1ToC2 = routine.trajectory("C1ToC2");
		
		routine.anyActive(ampToM1, m1ToC1, scoreToC2, c1ToC2).whileTrue(
			Commands.print("Intaking!!!! 1").withName("intake")
		);
		routine.anyActive(c1ToScore, c2ToScore).whileTrue(
			Commands.print("Aiming! 1").withName("Intake")
		);
		routine.active().onTrue(
			routine.resetOdometry(ampToM1).andThen(ampToM1.cmd()).withName("FourPieceAutoEntry")
		);
		
		ampToM1.done().onTrue(
			Commands.print("shooting! 1").andThen(m1ToC1.cmd())
		);
		
		m1ToC1.done().and(this::hasNote).onTrue(c1ToScore.cmd());
		m1ToC1.done().and(() -> !hasNote()).onTrue(c1ToC2.cmd());
		c1ToScore.done().onTrue(
			Commands.print("shooting! 2").andThen(scoreToC2.cmd())
		);
		
		scoreToC2.done().onTrue(Commands.print("HI!!").andThen(c2ToScore.cmd()));
		c2ToScore.done().onTrue(Commands.print("shooting! 3"));
		
		// alternative branches
		c1ToC2.done().onTrue(c2ToScore.cmd());
		
		return routine.cmd();
	}
}
