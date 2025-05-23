package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.utils.AllianceUtil;
import frc.robot.constants.TargetPoses;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;
import org.jetbrains.annotations.Nullable;

@RequiredArgsConstructor
public class AutoCommands {
	private final RobotCommands botCommands;
	private final AutoFactory autoFactory;
	private final CoralIntake coralIntake;
	private final SwerveDrive drivetrain;
	private final TargetPoses targetPoses;
	private final SwerveSetpointGenerator setpointGen;
	
	/* Utility classes/methods */
	
	/**
	 * Represents the source location when looking at the choreo field map.
	 * Here, TOP represents the source to the east from the driver station point of view,
	 * while BOTTOM represents the source to the west.
	 */
	private enum SourceLoc {
		TOP, BOTTOM
	}
	
	/**
	 * An auto "step" that represents scoring a piece of coral.
	 * @param level L1-L4(as an int).
	 * @param position The scoring position. See PathfindingPoses.java for more details.
	 * @param extendDelay Once the score trajectory starts,
	 *                    wait this long before extending the elevator & wrist.
	 * @param shouldAlign Whether to use PID to align to the reef during the scoring step.
	 */
	private record ScoringStep(int level, int position, double extendDelay, boolean shouldAlign) {
		public ScoringStep(int level, int position, double extendDelay) {
			this(level, position, extendDelay, level != 1); // Align is unecessary for L1
		}
	}
	
	/**
	 * An auto "step" that represents intaking from the human player station.
	 * @param sourceLoc the source to go to
	 * @param extendDelay Once the intake trajectory starts, wait this long before extending the wrist.
	 */
	private record IntakeStep(SourceLoc sourceLoc, double extendDelay) {}
	
	/** A combined intake and scoring step. */
	private record CombinedStep(IntakeStep intakeStep, ScoringStep scoringStep) {}
	
	/**
	 * Creates a generic auto routine.
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
		var previousTarget = targetPoses.reefBlue[initialScoreStep.position];
		for (var autoStep: otherSteps) {
			var intakeStep = autoStep.intakeStep;
			var scoringStep = autoStep.scoringStep;
			var sourceLoc = intakeStep.sourceLoc == SourceLoc.BOTTOM ? "SourceS" : "SourceN";
			var intakeTraj = routine.trajectory("Reef" + previousScoreStep.position + "To" + sourceLoc);
			var scoringTraj = routine.trajectory(sourceLoc + "ToReef" + scoringStep.position);
			
			if (previousScoreStep.shouldAlign) {
				var goal = previousTarget; // we create a temp variable because the () -> ... expression cannot use variables that change
				previousTraj.atTranslation(goal.getTranslation(), 0.3).onTrue(
					drivetrain.pathfindCmd(() -> AllianceUtil.flipIfRed(goal), setpointGen)
						.withTimeout(3)
						.andThen(Commands.print("AUTOALIGN DONE"), botCommands.waitUntilReady(), intakeTraj.spawnCmd())
						.withName("intake traj spawner")
				);
			} else {
				previousTraj.done().onTrue(
					botCommands.waitUntilReady()
						.andThen(Commands.waitUntil(coralIntake.hasCoral.negate()), intakeTraj.spawnCmd())
						.withName("intake traj spawner")
				);
			}
			intakeTraj.atTime(intakeStep.extendDelay).onTrue(botCommands.sourceIntake());
			intakeTraj.done().onTrue(
				Commands.waitUntil(coralIntake.hasCoral).andThen(scoringTraj.spawnCmd())
					.withName("scoring traj spawner")
			);
			scoringTraj.atTime(scoringStep.extendDelay).onTrue(
				botCommands.scoreSequence(scoringStep.level)
			);
			
			previousTraj = scoringTraj;
			previousScoreStep = scoringStep;
			previousTarget = targetPoses.reefBlue[previousScoreStep.position];
		}
		var taxiCmd = taxiTrajectory == null ? Commands.none() : taxiTrajectory.spawnCmd();
		if (previousScoreStep.shouldAlign) {
			var goal = previousTarget;
			previousTraj.atTranslation(previousTarget.getTranslation(), 0.2)
				.onTrue(
					drivetrain.pathfindCmd(() -> AllianceUtil.flipIfRed(goal), setpointGen)
						.andThen(botCommands.waitUntilReady(), taxiCmd)
						.withName("taxi traj spawner")
				);
		} else {
			previousTraj.done().onTrue(Commands.waitUntil(coralIntake.hasCoral.negate()).andThen(taxiCmd));
		}
		return routine.cmd();
	}
	
	/* Auto Routines */
	
	/** Scores 3 L4 coral in auto. */
	public Command tripleL4South() {
		var routine = autoFactory.newRoutine("TripleL4South");
		var intakeStep = new IntakeStep(SourceLoc.BOTTOM, 0.6);
		return genericAuto(
			routine, routine.trajectory("Reef10TaxiShort"),
			new ScoringStep(4, 9, 0.75, true),
			new CombinedStep(intakeStep, new ScoringStep(4, 11, 0.8)),
			new CombinedStep(intakeStep, new ScoringStep(4, 10, 0.8))
		);
	}
	
	/** Scores 4 L1 coral in auto. */
	public Command quadL1South() {
		var routine = autoFactory.newRoutine("QuadL1South");
		var intakeStep = new IntakeStep(SourceLoc.BOTTOM, 0.3);
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
		var intakeStep = new IntakeStep(SourceLoc.BOTTOM, 0.6);
		return genericAuto(
			routine, null,
			new ScoringStep(4, 9, 0.5, false),
			new CombinedStep(intakeStep, new ScoringStep(1, 10, 0.7)),
			new CombinedStep(intakeStep, new ScoringStep(1, 11, 0.8))
		);
	}
	
	/** Scores 2 L4 Coral and 1 L1 coral in auto. */
	public Command l4L4L1South() {
		var routine = autoFactory.newRoutine("L4L4L1South");
		var intakeStep = new IntakeStep(SourceLoc.BOTTOM, 0.6);
		return genericAuto(
			routine, routine.trajectory("Reef10TaxiLong"),
			new ScoringStep(4, 7, 0),
			new CombinedStep(intakeStep, new ScoringStep(4, 11, 0.7)),
			new CombinedStep(intakeStep, new ScoringStep(1, 10, 0.8))
		);
	}
	
	public Command onePieceL4(boolean mirrored) {
		int position = mirrored ? 5 : 9;
		var routine = autoFactory.newRoutine("OnePieceL4");
		return genericAuto(routine, null, new ScoringStep(4, position, 0.6, true));
	}
	
	public Command onePieceL4Center() {
		return genericAuto(autoFactory.newRoutine("OnePieceL4"), null, new ScoringStep(4, 7, 0.5));
	}
	
	public Command onePieceL1() {
		var routine = autoFactory.newRoutine("OnePieceL1");
		return genericAuto(routine, null, new ScoringStep(1, 9, 0.3, false));
	}
	
	public Command simplePath() {
		return autoFactory.resetOdometry("SimplePath")
			       .andThen(autoFactory.trajectoryCmd("SimplePath"));
	}
	
	public Command simplePathWithRotate() {
		return autoFactory.resetOdometry("SimplePathWithRotate")
			       .andThen(autoFactory.trajectoryCmd("SimplePathWithRotate"));
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
	
	public Command resetOdoTest() {
		return autoFactory.resetOdometry("LineToReef9");
	}
	
	public Command figureEight() {
		var routine = autoFactory.newRoutine("FigureEight");
		var figureEight = routine.trajectory("FigureEight");

		routine.active().onTrue(
			figureEight.resetOdometry().andThen(figureEight.spawnCmd())
		);

		return routine.cmd();
	}
	
	public Command taxi() {
		return drivetrain.driveCmd(() -> 0.3, () -> 0, () -> 0, false)
			       .withTimeout(5);
	}
}
