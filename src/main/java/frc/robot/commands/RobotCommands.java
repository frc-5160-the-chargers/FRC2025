package frc.robot.commands;

import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
	
	private Trigger readyToScore(Pose2d blueTargetPose) {
		return new Trigger(() -> {
			boolean almostAtTarget = distanceBetween(
				AllianceUtil.flipIfRed(blueTargetPose),
				drivetrain.poseEstimate()
			) < 0.2;
			return almostAtTarget && drivetrain.getOverallSpeedMPS() < 0.2;
		});
	}
	
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
			       .withName("l" + scoringLevel + "ScoreSequence");
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
			moveTo(Setpoint.INTAKE),
			coralIntake.intakeCmd()
		).withName("sourceIntake");
	}
	
	public Command moveToDemoSetpoint() {
		return Commands.parallel(
			coralIntakePivot.setDemoAngleCmd(),
			elevator.moveToDemoHeightCmd()
		).withName("moveToDemoSetpoint");
	}
}
