// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.subsystems.ExampleSubsystem

class Robot : TimedRobot() {
    private val autonomousCommand = PrintCommand("Hello, autonomous!")
    private val exampleSubsystem = ExampleSubsystem(1, 2)

    init {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin)
    }

    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        autonomousCommand.schedule()
    }

    override fun teleopInit() {
        autonomousCommand.cancel()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}
