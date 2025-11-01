package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.commands.NonBlockingCmds;
import frc.chargers.data.CurrAlliance;
import frc.robot.constants.ChoreoVars;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;

/**
 * Competition robot-specific commands that need more than one subsystem.
 */
public class Superstructure {
    private final SwerveDrive drivetrain;
    private final Intake intake;
    private final Wrist wrist;
    private final Elevator elevator;

    private Transform3d robotCenterToPivot = Transform3d.kZero;
    private final Transform3d pivotToCoralPosition = new Transform3d(0.25, 0, 0.07, new Rotation3d(Degrees.zero(), Degrees.of(10), Degrees.zero()));
    private final List<Pose3d> coralOuttakePositions = new ArrayList<>();

    public Superstructure(SwerveDrive drivetrain, Intake intake, Wrist wrist, Elevator elevator) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.wrist = wrist;
        this.elevator = elevator;
        if (RobotBase.isSimulation()) visualizationInit();
    }

    /**
     * A command that waits until the intake is not holding coral,
     * the elevator is moving down, and the elevator is at a low position.
     */
    public Command waitUntilReady() {
        return Commands.waitUntil(
            intake.isOuttaking.negate()
                .and(elevator.cogLow)
                .and(elevator.movingUp.negate())
        ).withName("wait until ready");
    }

    /**
     * Moves the elevator and pivot to a certain position.
     * @param setpoint the setpoint to move to - specifies elevator height and pivot angle.
     */
    public Command moveTo(Setpoint setpoint) {
        return NonBlockingCmds.sequence(
            Commands.runOnce(() -> Logger.recordOutput("setpoint", setpoint.name())),
            wrist.setAngleCmd(Setpoint.Limits.WRIST_LIMIT).withTimeout(1.5),
            elevator.moveToHeightCmd(setpoint.elevatorHeight()),
            wrist.setAngleCmd(setpoint.wristTarget())
        )
            .withTimeout(3.5)
            .withName("move to setpoint");
    }

    /** Moves the elevator and pivot to a stow position. */
    public Command stow() {
        return NonBlockingCmds.parallel(
            Commands.runOnce(() -> Logger.recordOutput("setpoint", "stow")),
            Commands.waitSeconds(0.5)
                .andThen(elevator.moveToHeightCmd(Setpoint.STOW.elevatorHeight())),
            wrist.setAngleCmd(Setpoint.Limits.STOW_WRIST_LIMIT)
                .until(elevator.cogLow)
                .andThen(wrist.setAngleCmd(Setpoint.STOW.wristTarget()))
        ).withName("stow");
    }

    /**
     * A complete scoring sequence(move to position, outtake, then move back). Only use in auto.
     * @param scoringLevel the level to score on(L1-L4)
     */
    public Command scoreSequence(int scoringLevel) {
        return NonBlockingCmds.sequence(
            moveTo(Setpoint.score(scoringLevel)),
            Commands.waitUntil(() -> drivetrain.overallSpeedMps() < 0.15)
                .withTimeout(3),
            Commands.waitSeconds(0.5),
            intake.outtakeCmd(),
            stow()
        ).withName("ScoreSequence(L" + scoringLevel + ")");
    }

    /** Moves the robot to the source intake position and runs the coral intake. */
    public Command sourceIntake() {
        return NonBlockingCmds.sequence(
            Commands.waitUntil(() -> elevator.heightMeters() < Setpoint.Limits.MIN_HEIGHT_BEFORE_INTAKE.in(Meters)),
            NonBlockingCmds.parallel(
                moveTo(Setpoint.INTAKE),
                intake.intakeCmd()
            )
        ).withName("source intake(no aim)");
    }

    /** Moves to a setpoint specified by tunable dashboard values. */
    public Command moveToDemoSetpoint() {
        return NonBlockingCmds.sequence(
            wrist.setDemoAngleCmd(),
            elevator.moveToDemoHeightCmd()
        ).withName("move to demo setpoint");
    }

    /** Renders the robot visualization. Must be called periodically. */
    public void visualizationPeriodic() {
        // logs relative positions for advantagescope visualization
        double currentHeight = elevator.heightMeters();
        Logger.recordOutput("RobotViz/stage1Position", Pose3d.kZero);
        Logger.recordOutput("RobotViz/stage2Position", new Pose3d(0, 0, MathUtil.clamp(currentHeight - 0.4, 0.0, 0.65), Rotation3d.kZero));
        Logger.recordOutput("RobotViz/stage3Position", new Pose3d(0, 0, currentHeight, Rotation3d.kZero));
        robotCenterToPivot = new Transform3d(
            0.374, -1.1875, 0.706 + currentHeight,
            new Rotation3d(0, wrist.getInputs().positionRad - degreesToRadians(10), 0)
        );
        Logger.recordOutput("RobotViz/intakePivotPosition", Pose3d.kZero.plus(robotCenterToPivot));
        Logger.recordOutput(
            "RobotViz/heldCoralPosition",
            intake.hasCoral.getAsBoolean()
                ? new Pose3d(drivetrain.bestPose())
                .plus(robotCenterToPivot)
                .plus(pivotToCoralPosition)
                : Pose3d.kZero
        );

        if (RobotBase.isSimulation()) {
            Logger.recordOutput("RobotViz/fieldCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            Logger.recordOutput("RobotViz/fieldAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
            Logger.recordOutput("CoralOuttakePositions", coralOuttakePositions.toArray(new Pose3d[0]));
        }
    }

    private void visualizationInit() {
        SimulatedArena.getInstance().placeGamePiecesOnField();

        intake.hasCoral
            .and(() -> intake.getInputs().velocityRadPerSec > 1)
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
            var eastS = ChoreoVars.Poses.eastSource;
            var westS = ChoreoVars.Poses.westSource;
            if (CurrAlliance.red()) {
                eastS = flip(eastS);
                westS = flip(westS);
            }
            var distFromEastSource = driveTrans.getDistance(eastS.getTranslation());
            var distFromWestSource = driveTrans.getDistance(westS.getTranslation());
            Logger.recordOutput("RobotViz/distFromEastSource", distFromEastSource);
            Logger.recordOutput("RobotViz/distFromWestSource", distFromWestSource);
            return (distFromEastSource < 1.3 || distFromWestSource < 1.3)
                && drivetrain.overallSpeedMps() < 0.1
                && Math.abs(intake.getInputs().velocityRadPerSec) > 30;
        })
            .onTrue(
                Commands.waitSeconds(1.0)
                    .andThen(intake.setHasCoralInSimCmd(true))
                    .withName("visualize coral intake")
            );
    }

    private Command visualizeCoralOuttakeCmd() {
        return intake.setHasCoralInSimCmd(false).andThen(
            Commands.runOnce(() -> {
                var robotCenterToCoral = robotCenterToPivot.plus(pivotToCoralPosition);
                SimulatedArena.getInstance().addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                        drivetrain.bestPose().getTranslation(),
                        robotCenterToCoral.getTranslation().toTranslation2d(),
                        drivetrain.robotRelativeSpeeds(),
                        drivetrain.bestPose().getRotation(),
                        robotCenterToCoral.getMeasureZ(),
                        MetersPerSecond.of(0.5),
                        Radians.of(wrist.getInputs().positionRad)
                    )
                );
                coralOuttakePositions.add(new Pose3d(drivetrain.bestPose()).plus(robotCenterToCoral));
            })
        ).withName("visualize coral outtake(sim only)");
    }
}