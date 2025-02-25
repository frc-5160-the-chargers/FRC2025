package frc.robot.commands;

import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.utils.AllianceUtil;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;

import static frc.chargers.utils.UtilMethods.distanceBetween;
import static monologue.Monologue.GlobalLog;

/**
 * Competition robot-specific commands that need more than one subsystem.
 * Example usage:
 * <pre><code>
 * RobotCommands robotCommands = new RobotCommands(...);
 * robotCommands.moveTo(Setpoint.score(3)) // moves arm and elevator to L3
 * robotCommands.scoreSequence(3) // moves arm and elevator to L3, auto-outtakes, and stows. Only used in auto
 * // pathfind to reef position 4, then move arm + elevator
 * robotCommands.pathfindAndMoveTo(Setpoint.score(3), pathfindingPoses.reefBlue[4])
 * </code></pre>
 */
@RequiredArgsConstructor
public class RobotCommands {
	private final SwerveDrive drivetrain;
	private final CoralIntake coralIntake;
	private final CoralIntakePivot coralIntakePivot;
	private final Elevator elevator;
	private final SwerveSetpointGenerator setpointGen;
	
	/**
	 * A command that waits until the intake is not holding coral,
	 * the elevator is moving down, and the elevator is at a low position.
	 */
	public Command waitUntilReady() {
		return Commands.waitUntil(
			coralIntake.hasCoral.negate()
				.and(elevator.atLowPosition)
				.and(elevator.movingUp.negate())
		);
	}
	
	/**
	 * Moves the elevator and pivot to a certain position.
	 * @param setpoint the setpoint to move to - specifies elevator height and pivot angle.
	 */
	public Command moveTo(Setpoint setpoint) {
		return Commands.parallel(
			Commands.runOnce(() -> GlobalLog.log("currentSetpoint", setpoint)),
			elevator.moveToHeightCmd(setpoint.elevatorHeight()),
			coralIntakePivot.setAngleCmd(setpoint.wristTarget())
		).withName("moveToSetpoint");
	}
	
	/**
	 * A complete scoring sequence(move to position, outtake, then move back). Only use in auto.
	 * @param scoringLevel the level to score on(L1-L4)
	 */
	public Command scoreSequence(int scoringLevel) {
		return moveTo(Setpoint.score(scoringLevel))
			       .andThen(
					   Commands.waitUntil(() -> drivetrain.getOverallSpeedMPS() < 0.05),
					   coralIntake.outtakeCmd(),
				       moveTo(Setpoint.STOW_MID)
			       )
			       .withName("scoreSequence(L" + scoringLevel + ")");
	}
	
	/** A command that pathfinds to a pose while moving to a certain setpoint. */
	public Command pathfindAndMoveTo(Setpoint setpoint, Pose2d blueAlliancePose) {
		return Commands.parallel(
			drivetrain.pathfindCmd(blueAlliancePose, true, setpointGen),
			Commands.waitUntil(() -> {
				var distance = distanceBetween(AllianceUtil.flipIfRed(blueAlliancePose), drivetrain.poseEstimate());
				return distance < 0.2 && drivetrain.getOverallSpeedMPS() < 0.2;
			}).andThen(moveTo(setpoint))
		).withName("moveToSetpoint(and pathfind)");
	}
	
	/** Runs the intake and moves the elevator and pivot to the intake position. */
	public Command sourceIntake() {
		return Commands.parallel(
			moveTo(Setpoint.INTAKE),
			coralIntake.intakeCmd()
		).withName("sourceIntake");
	}
	
	/** Moves to a setpoint specified by tunable dashboard values. */
	public Command moveToDemoSetpoint() {
		return Commands.parallel(
			coralIntakePivot.setDemoAngleCmd(),
			elevator.moveToDemoHeightCmd()
		).withName("moveToDemoSetpoint");
	}
}
