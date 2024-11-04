// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation.isFMSAttached
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.chargers.framework.setAutoCommand
import frc.chargers.wpilibextensions.Cmd
import monologue.Logged
import monologue.Monologue
import monologue.Monologue.MonologueConfig

class Robot : TimedRobot(), Logged {
    init {
        /* Required setup: The following code below is required for robot setup. */
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin)
        Monologue.setupMonologue(this, "Robot", MonologueConfig().withFileOnly { isFMSAttached() })
        addPeriodic(Monologue::updateAll, 0.02)
        addPeriodic(CommandScheduler.getInstance()::run, 0.02)

        setAutoCommand { Cmd.none() }
    }
}
