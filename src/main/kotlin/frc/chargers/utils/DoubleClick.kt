@file:Suppress("unused")
package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc6328.DoublePressTracker

fun Trigger.withDoubleClick(clickTimeout: Time? = null): Trigger =
    if (clickTimeout == null){
        DoublePressTracker.createTrigger(this)
    }else{
        DoublePressTracker.createTrigger(this, clickTimeout.inUnit(seconds))
    }

fun Trigger.onDoubleClick(command: Command, clickTimeout: Time? = null){
    withDoubleClick(clickTimeout).onTrue(command)
}

fun Trigger.onClickAndHold(command: Command, clickTimeout: Time? = null){
    withDoubleClick(clickTimeout).whileTrue(command)
}

