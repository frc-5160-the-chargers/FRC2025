package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.CoralIntakePivot.PivotAccel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.*;
import static monologue.Monologue.GlobalLog;

/**
 * Competition robot-specific commands that need more than one subsystem.
 * Example usage:
 * <pre><code>
 * RobotCommands botCommands = new RobotCommands(...);
 * botCommands.moveTo(Setpoint.score(3)) // moves arm and elevator to L3
 * botCommands.scoreSequence(3) // moves arm and elevator to L3, auto-outtakes, and stows. Only used in auto
 * // pathfind to reef position 4, then move arm + elevator
 * botCommands.pathfindAndMoveTo(Setpoint.score(3), pathfindingPoses.reefBlue[4])
 * </code></pre>
 */
@RequiredArgsConstructor
public class RobotCommands {
	private final SwerveDrive drivetrain;
	private final CoralIntake coralIntake;
	private final CoralIntakePivot coralIntakePivot;
	private final Elevator elevator;
	
	/**
	 * A command that waits until the intake is not holding coral,
	 * the elevator is moving down, and the elevator is at a low position.
	 */
	public Command waitUntilReady() {
		return Commands.waitUntil(
			coralIntake.isOuttaking.negate()
				.and(elevator.atLowPosition)
				.and(elevator.movingUp.negate())
		).withName("wait until ready");
	}

	public Command moveTo(Setpoint setpoint) {
		return moveTo(setpoint, PivotAccel.FAST);
	}
	
	/**
	 * Moves the elevator and pivot to a certain position.
	 * @param setpoint the setpoint to move to - specifies elevator height and pivot angle.
	 * @param pivotAccel whether to move the pivot with a fast or slow acceleration.
	 */
	public Command moveTo(Setpoint setpoint, PivotAccel pivotAccel) {
		return Commands.runOnce(() -> GlobalLog.log("setpoint", setpoint.name()))
			       .andThen(
						coralIntakePivot.setAngleCmd(Setpoint.Limits.WRIST_LIMIT, pivotAccel)
			                .until(() -> coralIntakePivot.angleRads() >= Setpoint.Limits.WRIST_LIMIT.in(Radians)),
						coralIntakePivot.stopCmd(),
						elevator.moveToHeightCmd(setpoint.elevatorHeight()),
						coralIntakePivot.setAngleCmd(setpoint.wristTarget())
			       )
			       .withTimeout(3.5)
			       .withName("move to setpoint");
	}
	
	/** Moves the elevator and pivot to a stow position. */
	public Command stow() {
		return Commands.runOnce(() -> GlobalLog.log("setpoint", "stow"))
			.andThen(
				coralIntakePivot.setAngleCmd(Setpoint.Limits.STOW_WRIST_LIMIT, PivotAccel.SLOW),
				Commands.parallel(
					elevator.moveToHeightCmd(Setpoint.STOW.elevatorHeight()),
					coralIntakePivot.idleCmd()
						.until(elevator.canMovePivotIn)
						.andThen(coralIntakePivot.setAngleCmd(Setpoint.STOW.wristTarget()))
				)
			)
			.withName("stow");
	}
	
	/**
	 * A complete scoring sequence(move to position, outtake, then move back). Only use in auto.
	 * @param scoringLevel the level to score on(L1-L4)
	 */
	public Command scoreSequence(int scoringLevel) {
		return moveTo(Setpoint.score(scoringLevel))
			       .andThen(
					   Commands.deadline(
						   Commands.waitUntil(() -> drivetrain.getOverallSpeedMPS() < 0.15)
							   .withTimeout(3)
					           .andThen(Commands.waitSeconds(0.5), coralIntake.outtakeCmd()),
						   coralIntakePivot.idleCmd() // keeps it steady by counteracting gravity
					   ),
				       stow()
			       )
			       .finallyDo(coralIntakePivot::requestStop)
			       .withName("score sequence(L" + scoringLevel + ")");
	}
	
	/** Moves the robot to the source intake position and runs the coral intake. */
	public Command sourceIntake() {
		return coralIntakePivot.idleCmd()
			.until(() -> elevator.heightMeters() < Setpoint.Limits.MIN_HEIGHT_BEFORE_INTAKE.in(Meters))
			.andThen(
				Commands.parallel(
					moveTo(Setpoint.INTAKE, PivotAccel.SLOW),
					coralIntake.intakeCmd()
				)
			)
			.withName("source intake(no aim)");
	}
	
	/** Moves to a setpoint specified by tunable dashboard values. */
	public Command moveToDemoSetpoint() {
		return Commands.parallel(
			coralIntakePivot.setDemoAngleCmd(),
			Commands.waitUntil(() -> coralIntakePivot.angleRads() >= Setpoint.Limits.WRIST_LIMIT.in(Radians))
		        .andThen(elevator.moveToDemoHeightCmd())
		).withName("move to demo setpoint");
	}
}
