package frc.robot.commands;

import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.AllianceUtil;
import frc.robot.constants.PathfindingPoses;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.chargers.utils.UtilMethods.distanceBetween;
import static monologue.Monologue.GlobalLog;

/**
 * Commands that need more than one subsystem.
 * Example usage:
 * <pre><code>
 * RobotCommands robotCommands = new RobotCommands(...);
 * robotCommands.moveTo(Setpoint.score(3)) // moves arm and elevator to L3
 * robotCommands.scoreSequence(3) // moves arm and elevator to L3, auto-outtakes, and stows. Only used in auto
 * // pathfind to reef position 4, then move arm + elevator
 * robotCommands.pathfindAndMoveTo(Setpoint.score(3), PathfindingPoses.reef(4))
 * </code></pre>
 */
@RequiredArgsConstructor
public class RobotCommands {
	private final SwerveDrive drivetrain;
	private final CoralIntake coralIntake;
	private final CoralIntakePivot coralIntakePivot;
	private final Elevator elevator;
	private final SwerveSetpointGenerator setpointGen;
	private final PathfindingPoses pathfindingPoses;
	
	private Trigger readyToScore(Pose2d blueTargetPose) {
		return new Trigger(() -> {
			boolean almostAtTarget = distanceBetween(
				AllianceUtil.flipIfRed(blueTargetPose),
				drivetrain.poseEstimate()
			) < 0.2;
			return almostAtTarget && drivetrain.getOverallSpeed().in(MetersPerSecond) < 0.2;
		});
	}
	
	public Command moveTo(Setpoint setpoint) {
		return Commands.parallel(
			Commands.runOnce(() -> GlobalLog.log("CurrentSetpoint", setpoint)),
			elevator.moveToHeightCmd(setpoint.elevatorHeight()),
			coralIntakePivot.setAngleCmd(setpoint.wristTarget())
		).withName("moveToSetpoint");
	}
	
	/** A complete scoring sequence(move to position, outtake, then move back). Only use in auto. */
	public Command scoreSequence(int scoringLevel) {
		return moveTo(Setpoint.score(scoringLevel))
			       .andThen(coralIntake.outtakeCmd(), moveTo(Setpoint.STOW))
			       .withName("scoreSequenceL" + scoringLevel);
	}
	
	public Command pathfindAndMoveTo(Setpoint setpoint, Pose2d blueAlliancePose) {
		return Commands.parallel(
			drivetrain.pathfindCmd(blueAlliancePose, true, setpointGen),
			Commands.waitUntil(readyToScore(blueAlliancePose))
				.andThen(moveTo(setpoint))
		).withName("moveToSetpoint(and pathfind)");
	}
	
	public Command sourceIntake() {
		return Commands.parallel(
			moveTo(Setpoint.SOURCE_INTAKE),
			coralIntake.intakeCmd()
		).withName("sourceIntake");
	}
	
	public Command pathfindAndIntakeNorthSource() {
		return Commands.parallel(
			pathfindAndMoveTo(Setpoint.SOURCE_INTAKE, pathfindingPoses.northSource()),
			coralIntake.intakeCmd()
		).withName("northSourcePathfindIntake");
	}
	
	public Command pathfindAndIntakeSouthSource() {
		return Commands.parallel(
			pathfindAndMoveTo(Setpoint.SOURCE_INTAKE, pathfindingPoses.southSource()),
			coralIntake.intakeCmd()
		).withName("southSourcePathfindIntake");
	}
	
	public Command moveToDemoSetpoint() {
		return Commands.parallel(
			coralIntakePivot.setDemoAngleCmd(),
			elevator.moveToDemoHeightCmd()
		).withName("MoveToDemoSetpoint");
	}
}
