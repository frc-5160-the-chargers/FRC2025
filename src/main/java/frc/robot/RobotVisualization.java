package frc.robot;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.AllianceUtil;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.LogLocal;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import static edu.wpi.first.units.Units.*;

/** A class for rendering poses for advantagescope visualization. */
public class RobotVisualization implements LogLocal {
	// @NotLogged disables monologue logging too
	@NotLogged private final SwerveDrive drivetrain;
	@NotLogged private final CoralIntake coralIntake;
	@NotLogged private final CoralIntakePivot coralIntakePivot;
	@NotLogged private final Elevator elevator;
	
	private Transform3d robotCenterToPivot = Transform3d.kZero;
	private final Transform3d pivotToCoralPosition = new Transform3d(0.3, 0, 0.1, Rotation3d.kZero);
	
	public RobotVisualization(SwerveDrive drivetrain, CoralIntake coralIntake, CoralIntakePivot coralIntakePivot, Elevator elevator) {
		this.drivetrain = drivetrain;
		this.coralIntake = coralIntake;
		this.coralIntakePivot = coralIntakePivot;
		this.elevator = elevator;
		
		if (RobotBase.isSimulation()) {
			coralIntake.hasCoral
				.and(() -> coralIntake.speedRadPerSec() > 0.5)
				.onTrue(visualizeCoralOuttakeCmd());
			
			// if close to either source(and stopped), simulate the robot getting a gamepiece
			new Trigger(() -> {
				var driveTrans = drivetrain.bestPose().getTranslation();
				var distFromNorthSource = driveTrans.getDistance(
					AllianceUtil.flipIfRed(FieldConstants.CoralStation.leftCenterFace.getTranslation())
				);
				var distFromSouthSource = driveTrans.getDistance(
					AllianceUtil.flipIfRed(FieldConstants.CoralStation.rightCenterFace.getTranslation())
				);
				return (distFromNorthSource < 0.8 || distFromSouthSource < 0.8)
					       && Math.abs(drivetrain.getOverallSpeed().in(MetersPerSecond)) < 0.5
						   && Math.abs(coralIntake.speedRadPerSec()) > 0.5;
			})
				.onTrue(coralIntake.setHasCoralInSimCmd(true));
		}
	}
	
	/** Renders the robot visualization. Must be called periodically. */
	public void render() {
		// logs relative positions for advantagescope visualization
		double currentHeight = elevator.extensionHeight();
		log("stage1Position", Pose3d.kZero);
		log("stage2Position", new Pose3d(0, 0, MathUtil.clamp(currentHeight - 0.4, 0.0, 0.65), Rotation3d.kZero));
		log("stage3Position", new Pose3d(0, 0, currentHeight, Rotation3d.kZero));
		robotCenterToPivot = new Transform3d(0.374, 0.173, 0.557 + currentHeight, new Rotation3d(0, -coralIntakePivot.angleRads(), 0));
		log("intakePivotPosition", Pose3d.kZero.plus(robotCenterToPivot));
		
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().simulationPeriodic();
			log("fieldCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
			log("fieldAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		}
		
		log(
			"heldCoralPosition",
			coralIntake.hasCoral.getAsBoolean()
				? new Pose3d(drivetrain.bestPose())
					  .plus(robotCenterToPivot)
					  .plus(pivotToCoralPosition)
				: Pose3d.kZero
		);
	}
	
	private Command visualizeCoralOuttakeCmd() {
		return Commands.waitSeconds(0.3).andThen(
			coralIntake.setHasCoralInSimCmd(false),
			Commands.runOnce(() -> {
		        var robotCenterToCoral = robotCenterToPivot.plus(pivotToCoralPosition).getTranslation();
		        SimulatedArena.getInstance().addGamePieceProjectile(
			        new ReefscapeCoralOnFly(
				        drivetrain.bestPose().getTranslation(),
				        robotCenterToCoral.toTranslation2d(),
				        drivetrain.getMeasuredSpeeds(),
				        drivetrain.bestPose().getRotation(),
				        robotCenterToCoral.getMeasureZ(),
				        MetersPerSecond.of(2),
				        Radians.of(coralIntakePivot.angleRads() - 0.2)
			        )
		        );
			})
       );
	}
}
