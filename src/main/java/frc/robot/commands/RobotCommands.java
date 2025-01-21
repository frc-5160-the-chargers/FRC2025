package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.field.ScoringLevel;
import frc.robot.field.ScoringPoses;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.field.IntakePosition;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static monologue.Monologue.GlobalLog;

/**
 * Commands that need more than one subsystem.
 */
@RequiredArgsConstructor
public class RobotCommands {
	private final SwerveDrive drivetrain;
	private final CoralIntake coralIntake;
	private final CoralIntakePivot coralIntakePivot;
	private final Elevator elevator;
	
	// Note: Commands.parallel runs parallel until everything in it finishes.
	public Command prepareToScore(ScoringLevel level) {
		return Commands.parallel(
			elevator.moveToHeightCmd(level.elevatorHeight()),
			Commands.runOnce(() -> GlobalLog.log("scoringPosition/scoringLevel", level.value)),
			coralIntakePivot.setAngleCmd(level.pivotAngle())
		).withName("prepareToScore");
	}
	
	/** A complete scoring sequence. Only use during autonomous. */
	public Command autoScore(ScoringLevel position) {
		return prepareToScore(position)
			       .andThen(coralIntake.outtakeCmd(), stow())
			       .withName("autoScore");
	}
	
	public Command pathfindThenPrepareScore(ScoringLevel position, int poseIndex) {
		var targetPose = ScoringPoses.getPose(poseIndex);
		return Commands.runOnce(() -> targetPose.ifPresent(it -> GlobalLog.log("targetPose", it)))
			       .andThen(
				       pathfindWithLift(
						   () -> targetPose.orElseGet(drivetrain::getPose),
					       position.elevatorHeight()
				       ),
					   prepareToScore(position)
			       )
			       .withName("pathfindThenPrepareScore");
	}
	
	public Command pathfindThenSourceIntake(IntakePosition position) {
		return Commands.parallel(
			drivetrain.pathfindCmd(() -> position.pose),
			sourceIntake(),
			Commands.runOnce(() -> GlobalLog.log("intakePosition", position.toString()))
		).withName("pathfindThenSourceIntake");
	}
	
	public Command sourceIntake() {
		return Commands.parallel(
			elevator.moveToHeightCmd(IntakePosition.ELEVATOR_HEIGHT),
			coralIntakePivot.setAngleCmd(IntakePosition.PIVOT_ANGLE),
			coralIntake.intakeCmd()
		).withName("sourceIntake");
	}
	
	public Command stow() {
		return Commands.parallel(
			elevator.moveToHeightCmd(Meters.of(0)),
			coralIntakePivot.setAngleCmd(Degrees.of(60))
		);
	}
	
	// A pathfind command variant that passively moves the elevator slightly along the way.
	// This slightly reduces cycle times(as the elevator is already in a higher position).
	// Note: CommandA.withDeadline(CommandB) runs A and B parallel and stops until B finishes.
	private Command pathfindWithLift(Supplier<Pose2d> getTargetPose, Distance elevatorHeight) {
		return elevator.passiveLiftCmd(elevatorHeight)
			       .withDeadline(drivetrain.pathfindCmd(getTargetPose));
	}
}
