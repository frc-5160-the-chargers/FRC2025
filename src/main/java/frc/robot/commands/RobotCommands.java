package frc.robot.commands;

import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.utils.AllianceUtil;
import frc.robot.components.controllers.DriverController;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.distanceBetween;
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
	private final SwerveSetpointGenerator setpointGen;
	
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
	
	/**
	 * Moves the elevator and pivot to a certain position.
	 * @param setpoint the setpoint to move to - specifies elevator height and pivot angle.
	 */
	public Command moveTo(Setpoint setpoint) {
		return Commands.parallel(
			Commands.runOnce(() -> GlobalLog.log("setpoint", setpoint.name())),
			coralIntakePivot.setAngleCmd(setpoint.wristTarget()),
			Commands.waitUntil(() -> coralIntakePivot.angleRads() >= Setpoint.Limits.WRIST_LIMIT.in(Degrees))
				.andThen(elevator.moveToHeightCmd(setpoint.elevatorHeight()))
		).withName("move to setpoint");
	}
	
	/** Moves the elevator and pivot to a stow position. */
	public Command stow() {
		BooleanSupplier elevatorLow = () -> elevator.heightMeters() <= Setpoint.Stow.ELEVATOR_THRESHOLD.in(Meters);
		BooleanSupplier wristAtThreshold = () -> coralIntakePivot.angleRads() <= Setpoint.Stow.WRIST_THRESHOLD_1.in(Radians);
		return Commands.runOnce(() -> GlobalLog.log("setpoint", "stow"))
			       .andThen(
					   // Negative wrist angle = up
					   coralIntakePivot.setAngleCmd(Setpoint.Stow.WRIST_TARGET_1)
						   .until(() -> wristAtThreshold.getAsBoolean() || elevatorLow.getAsBoolean()),
				       Commands.parallel(
						   elevator.moveToHeightCmd(Setpoint.Stow.ELEVATOR_HEIGHT),
						   coralIntakePivot.idleCmd()
							   .until(elevatorLow)
						       .andThen(coralIntakePivot.setAngleCmd(Setpoint.Stow.WRIST_TARGET_2))
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
					   Commands.waitUntil(() -> drivetrain.getOverallSpeedMPS() < 0.05),
					   Commands.waitSeconds(0.3),
					   coralIntake.outtakeCmd(),
				       stow()
			       )
			       .withName("score sequence(L" + scoringLevel + ")");
	}
	
	/** A command that pathfinds to a pose while moving to a certain setpoint. */
	public Command pathfindAndMoveTo(Setpoint setpoint, Pose2d blueAlliancePose) {
		return Commands.parallel(
			drivetrain.pathfindCmd(blueAlliancePose, true, setpointGen),
			Commands.waitUntil(() -> {
				var distance = distanceBetween(AllianceUtil.flipIfRed(blueAlliancePose), drivetrain.poseEstimate());
				return distance < 0.2 && drivetrain.getOverallSpeedMPS() < 0.2;
			}).andThen(moveTo(setpoint))
		).withName("move to setpoint(and pathfind)");
	}
	
	public Command aimAndMoveTo(
		Setpoint setpoint,
		Pose2d blueAlliancePose,
		DriverController controller,
		double maxRotationSpeed
	) {
		return Commands.parallel(
			moveTo(setpoint),
			drivetrain.driveWithAimCmd(
				controller.forwardOutput, controller.strafeOutput,
				AllianceUtil.flipIfRed(blueAlliancePose.getRotation()).getMeasure(),
				maxRotationSpeed,
				true
			)
		).withName("move to setpoint(and aim)");
	}
	
	/** Runs the intake and moves the elevator and pivot to the intake position. */
	public Command aimAndSourceIntake(Pose2d sourcePoseBlue, DriverController controller) {
		return Commands.parallel(
			aimAndMoveTo(Setpoint.INTAKE, sourcePoseBlue, controller, 0.2),
			coralIntake.intakeCmd()
		).withName("source intake with aim");
	}
	
	public Command sourceIntake() {
		return Commands.parallel(moveTo(Setpoint.INTAKE), coralIntake.intakeCmd())
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
