package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.positions.IntakePosition;
import frc.robot.positions.ScoringPosition;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

import static monologue.Monologue.GlobalLog;

/**
 * Commands that need more than one subsystem.
 */
@RequiredArgsConstructor
public class CmdFactory {
	private final SwerveDrive drivetrain;
	private final CoralIntake coralIntake;
	private final CoralIntakePivot coralIntakePivot;
	private final Elevator elevator;
	
	private void logScorePos(ScoringPosition position) {
		GlobalLog.log("scoringPosition/scoringLevel", position.scoringLevel());
		GlobalLog.log("scoringPosition/nodePosition", position.nodePosition());
	}
	
	public Command score(ScoringPosition position) {
		return elevator.moveToHeightCmd(position.elevatorHeight())
			       .alongWith(coralIntakePivot.setAngleCmd(position.pivotAngle()))
			       .beforeStarting(() -> logScorePos(position))
			       .withName("score");
	}
	
	public Command scoreThenOuttake(ScoringPosition position) {
		return score(position).andThen(coralIntake.outtakeCmd());
	}
	
	public Command pathfindThenScore(ScoringPosition position) {
		return pathfindWithLift(position::pathfindingTarget, position.elevatorHeight())
			       .andThen(score(position))
			       .beforeStarting(() -> logScorePos(position))
			       .withName("pathfindThenScore");
	}
	
	public Command intakeWithPathfind(IntakePosition position) {
		return drivetrain.pathfindCmd(() -> position.pose)
			       .alongWith(
					   intake(),
				       Commands.runOnce(() -> GlobalLog.log("intakePosition", position.toString()))
			       )
			       .withName("pathfindThenSourceIntake");
	}
	
	public Command intake() {
		return elevator.moveToHeightCmd(IntakePosition.ELEVATOR_HEIGHT)
			       .alongWith(
					   coralIntakePivot.setAngleCmd(IntakePosition.PIVOT_ANGLE),
				       coralIntake.intakeCmd()
			       )
			       .withName("sourceIntake");
	}
	
	// A pathfind command variant that passively moves the elevator slightly along the way.
	// This slightly reduces cycle times(as the elevator is already in a higher position).
	private Command pathfindWithLift(Supplier<Pose2d> getTargetPose, Distance elevatorHeight) {
		return Commands.parallel(drivetrain.pathfindCmd(getTargetPose))
			       .withDeadline(elevator.passiveLiftCmd(elevatorHeight));
	}
}
