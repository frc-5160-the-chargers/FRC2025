package frc.chargers.framework

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.chargers.wpilibextensions.Cmd

/**
 * Sets the robot's auto command.
 */
fun setAutoCommand(autoCommandSupplier: () -> Command) {
    RobotModeTriggers.autonomous().whileTrue(Cmd.defer(autoCommandSupplier, setOf()))
}

/**
 * Sets the robot's test command.
 */
fun setTestCommand(testCommandSupplier: () -> Command) {
    RobotModeTriggers.test().whileTrue(Cmd.defer(testCommandSupplier, setOf()))
}