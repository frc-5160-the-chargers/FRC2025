package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.commands.NonBlockingCmds;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

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
    private final Intake intake;
    private final Wrist wrist;
    private final Elevator elevator;

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
}