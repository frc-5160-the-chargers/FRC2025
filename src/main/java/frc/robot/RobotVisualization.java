package frc.robot;

import edu.wpi.first.epilogue.Logged;
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
import frc.robot.constants.OtherConstants;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.LogLocal;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;

/** A class for rendering poses for advantagescope visualization. */
public class RobotVisualization implements LogLocal {
	// @NotLogged disables monologue logging too
	@NotLogged private final SwerveDrive drivetrain;
	@NotLogged private final CoralIntake coralIntake;
	@NotLogged private final CoralIntakePivot coralIntakePivot;
	@NotLogged private final Elevator elevator;
	
	private Transform3d robotCenterToPivot = Transform3d.kZero;
	private final Transform3d pivotToCoralPosition = new Transform3d(0.25, 0, 0.07, new Rotation3d(Degrees.zero(), Degrees.of(10), Degrees.zero()));
	@Logged private final List<Pose3d> coralOuttakePositions = new ArrayList<>();
	
	public RobotVisualization(SwerveDrive drivetrain, CoralIntake coralIntake, CoralIntakePivot coralIntakePivot, Elevator elevator) {
		this.drivetrain = drivetrain;
		this.coralIntake = coralIntake;
		this.coralIntakePivot = coralIntakePivot;
		this.elevator = elevator;
		
		if (RobotBase.isSimulation()) {
			coralIntake.hasCoral
				.and(() -> coralIntake.velocityRadPerSec() > 1)
				.onTrue(visualizeCoralOuttakeCmd());
			
			// in auto or test mode, clear on-field coral/algae
			autonomous()
				.or(test())
				.onTrue(Commands.runOnce(() -> {
					SimulatedArena.getInstance().clearGamePieces();
					SimulatedArena.getInstance().placeGamePiecesOnField(); // re-place game pieces on field
					coralOuttakePositions.clear();
				}));
			
			// if close to either source(and stopped), simulate the robot getting a gamepiece
			new Trigger(() -> {
				var driveTrans = drivetrain.bestPose().getTranslation();
				var distFromEastSource = driveTrans.getDistance(
					AllianceUtil.flipIfRed(FieldConstants.CoralStation.leftCenterFace.getTranslation())
				);
				var distFromWestSource = driveTrans.getDistance(
					AllianceUtil.flipIfRed(FieldConstants.CoralStation.rightCenterFace.getTranslation())
				);
				log("distFromEastSource", distFromEastSource);
				log("distFromWestSource", distFromWestSource);
				return (distFromEastSource < 1.3 || distFromWestSource < 1.3)
					       && drivetrain.getOverallSpeedMPS() < 0.1
						   && Math.abs(coralIntake.velocityRadPerSec()) > 30;
			})
				.onTrue(
					Commands.waitSeconds(1.0)
						.andThen(coralIntake.setHasCoralInSimCmd(true))
						.withName("visualize coral intake")
				);
		}
	}
	
	/** Renders the robot visualization. Must be called periodically. */
	public void periodic() {
		// logs relative positions for advantagescope visualization
		double currentHeight = elevator.heightMeters();
		log("stage1Position", Pose3d.kZero);
		log("stage2Position", new Pose3d(0, 0, MathUtil.clamp(currentHeight - 0.4, 0.0, 0.65), Rotation3d.kZero));
		log("stage3Position", new Pose3d(0, 0, currentHeight, Rotation3d.kZero));
		robotCenterToPivot = new Transform3d(
			0.374, -OtherConstants.INTAKE_OFFSET_FROM_CENTER.in(Meters), 0.706 + currentHeight,
			new Rotation3d(0, coralIntakePivot.angleRads() - degreesToRadians(10), 0)
		);
		log("intakePivotPosition", Pose3d.kZero.plus(robotCenterToPivot));
		
		if (RobotBase.isSimulation()) {
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
		return coralIntake.setHasCoralInSimCmd(false).andThen(
			Commands.runOnce(() -> {
				var robotCenterToCoral = robotCenterToPivot.plus(pivotToCoralPosition);
				SimulatedArena.getInstance().addGamePieceProjectile(
					new ReefscapeCoralOnFly(
						drivetrain.bestPose().getTranslation(),
						robotCenterToCoral.getTranslation().toTranslation2d(),
						drivetrain.getMeasuredSpeeds(),
						drivetrain.bestPose().getRotation(),
						robotCenterToCoral.getMeasureZ(),
						MetersPerSecond.of(0.5),
						Radians.of(coralIntakePivot.angleRads())
					)
				);
				coralOuttakePositions.add(new Pose3d(drivetrain.bestPose()).plus(robotCenterToCoral));
			})
       ).withName("visualize coral outtake(sim only)");
	}
}
