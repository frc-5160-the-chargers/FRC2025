package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.ExampleSubsystem

/**
 * An example of a function returning an autonomous command.
 */
fun exampleAuto(exampleSubsystem: ExampleSubsystem) =
    Commands.sequence(
        exampleSubsystem.exampleMethodCommand(),
        ExampleCommand(exampleSubsystem)
    )