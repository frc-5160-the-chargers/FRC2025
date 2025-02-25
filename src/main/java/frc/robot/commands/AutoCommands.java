package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.PathfindingPoses;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;
import org.jetbrains.annotations.Nullable;

@RequiredArgsConstructor
public class AutoCommands {
	private final RobotCommands botCommands;
	private final AutoFactory autoFactory;
	private final CoralIntake coralIntake;
	private final Elevator elevator;
	private final SwerveDrive drivetrain;
	private final PathfindingPoses pathfindingPoses;
	
	/* Utility classes/methods */
	
	/**
	 * An auto "step" that represents scoring a piece of coral.
	 * @param level L1-L4(as an int).
	 * @param position The scoring position. See PathfindingPoses.java for more details.
	 * @param extendDelay Once the score trajectory starts,
	 *                    wait this long before extending the elevator & wrist.
	 */
	public record ScoringStep(int level, int position, double extendDelay) {}
	
	/**
	 * An auto "step" that represents intaking from the human player station.
	 * @param isSouthSource If false, intakes from the north source instead.
	 * @param extendDelay Once the intake trajectory starts, wait this long before extending the wrist.
	 */
	public record IntakeStep(boolean isSouthSource, double extendDelay) {}
	
	/** A combined intake and scoring step. */
	public record CombinedStep(IntakeStep intakeStep, ScoringStep scoringStep) {}
	
	/**
	 * Assigns a series of auto steps to an auto routine.
	 * To use this command, all trajectories must follow a standard naming format.
	 * 1. The initial trajectory must be named "LineToReefX", where X is the location
	 *      (an integer, specified in PathfindingPoses.java).
	 * 2. Intaking trajectories must be named "ReefXToSourceS"(south source)
	 *    or "ReefXToSourceN"(north source).
	 * 3. Scoring trajectories must be named "SourceSToReefX" or "SourceNToReefX".
	 */
	private Command genericAuto(
		AutoRoutine routine,
		@Nullable AutoTrajectory taxiTrajectory,
		ScoringStep initialScoreStep,
		CombinedStep... otherSteps
	) {
		var initialScoreTraj = routine.trajectory("LineToReef" + initialScoreStep.position);
		routine.active().onTrue(
			initialScoreTraj.resetOdometry().andThen(initialScoreTraj.spawnCmd())
		);
		initialScoreTraj.atTime(initialScoreStep.extendDelay).onTrue(
			botCommands.scoreSequence(initialScoreStep.level)
		);
		var previousScoreStep = initialScoreStep;
		var previousTraj = initialScoreTraj;
		var previousTarget = pathfindingPoses.reefBlue[initialScoreStep.position];
		for (var autoStep: otherSteps) {
			var intakeStep = autoStep.intakeStep;
			var scoringStep = autoStep.scoringStep;
			var sourceLoc = intakeStep.isSouthSource ? "SourceS" : "SourceN";
			var intakeTraj = routine.trajectory("Reef" + previousScoreStep.position + "To" + sourceLoc);
			var scoringTraj = routine.trajectory(sourceLoc + "ToReef" + scoringStep.position);
			
			if (previousScoreStep.level == 1) {
				previousTraj.done().onTrue(Commands.waitSeconds(1).andThen(intakeTraj.spawnCmd()));
			} else {
				previousTraj.atTranslation(previousTarget.getTranslation(), 0.4).onTrue(
					drivetrain.pathfindCmd(previousTarget, true, null)
						.andThen(botCommands.waitUntilReady(), intakeTraj.spawnCmd())
				);
			}
			intakeTraj.active().onTrue(coralIntake.intakeCmd());
			intakeTraj.atTime(intakeStep.extendDelay).onTrue(botCommands.moveTo(Setpoint.INTAKE));
			intakeTraj.done().onTrue(
				Commands.waitUntil(coralIntake.hasCoral).andThen(scoringTraj.spawnCmd())
			);
			scoringTraj.atTime(scoringStep.extendDelay).onTrue(
				botCommands.scoreSequence(scoringStep.level)
			);
			
			previousTraj = scoringTraj;
			previousScoreStep = scoringStep;
			previousTarget = pathfindingPoses.reefBlue[previousScoreStep.position];
		}
		var taxiCmd = taxiTrajectory == null ? Commands.none() : taxiTrajectory.spawnCmd();
		if (previousScoreStep.level == 1) {
			previousTraj.done().onTrue(Commands.waitSeconds(1).andThen(taxiCmd));
		} else {
			previousTraj.atTranslation(previousTarget.getTranslation(), 0.4)
				.onTrue(
					drivetrain.pathfindCmd(previousTarget, true, null)
						.andThen(Commands.waitUntil(elevator.atLowPosition), taxiCmd)
				);
		}
		return routine.cmd();
	}
	
	/* Auto Routines */
	
	/** Scores 3 L4 coral in auto. */
	public Command tripleL4South() {
		var routine = autoFactory.newRoutine("TripleL4South");
		var intakeStep = new IntakeStep(true, 0.6);
		return genericAuto(
			routine, routine.trajectory("Reef10TaxiShort"),
			new ScoringStep(4, 9, 0.75),
			new CombinedStep(intakeStep, new ScoringStep(4, 11, 0.7)),
			new CombinedStep(intakeStep, new ScoringStep(4, 10, 0.8))
		);
	}
	
	public Command quadL1South() {
		var routine = autoFactory.newRoutine("QuadL1South");
		var intakeStep = new IntakeStep(true, 0.3);
		return genericAuto(
			routine, null,
			new ScoringStep(1, 9, 0),
			new CombinedStep(intakeStep, new ScoringStep(1, 11, 0.7)),
			new CombinedStep(intakeStep, new ScoringStep(1, 10, 0.8)),
			new CombinedStep(intakeStep, new ScoringStep(1, 0, 0.8))
		);
	}
	
	/** Scores 1 L4 Coral and 2 L1 coral in auto. */
	public Command l4L1L1South() {
		var routine = autoFactory.newRoutine("L4L1L1South");
		var intakeStep = new IntakeStep(true, 0.6);
		return genericAuto(
			routine, routine.trajectory("Reef10TaxiLong"),
			new ScoringStep(4, 7, 0),
			new CombinedStep(intakeStep, new ScoringStep(1, 10, 0.7)),
			new CombinedStep(intakeStep, new ScoringStep(1, 11, 0.8))
		);
	}
	
	/** Scores 2 L4 Coral and 1 L1 coral in auto. */
	public Command l4L4L1South() {
		var routine = autoFactory.newRoutine("L4L4L1South");
		var intakeStep = new IntakeStep(true, 0.6);
		return genericAuto(
			routine, routine.trajectory("Reef10TaxiLong"),
			new ScoringStep(4, 7, 0),
			new CombinedStep(intakeStep, new ScoringStep(4, 11, 0.7)),
			new CombinedStep(intakeStep, new ScoringStep(1, 10, 0.8))
		);
	}
	
	public Command pathTest() {
		return autoFactory.resetOdometry("SimplePath").andThen(
			Commands.runOnce(() -> System.out.println("SKDJFLJKLJSLKDJFKLSJDKFJKSDLFKLSDJFKLSDJ")),
			autoFactory.trajectoryCmd("SimplePath")
		).withName("pathTest");
	}
	
	public Command resetOdometryTest() {
		return autoFactory.resetOdometry("LineToReef9");
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
