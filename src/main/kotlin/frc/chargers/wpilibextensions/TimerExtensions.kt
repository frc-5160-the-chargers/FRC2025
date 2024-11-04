@file:Suppress("unused")
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.Timer

fun fpgaTimestamp(): Time = Timer.getFPGATimestamp().ofUnit(seconds)

fun timeSinceMatchStart(): Time = Timer.getMatchTime().ofUnit(seconds)

fun delay(time: Time){
    Timer.delay(time.inUnit(seconds))
}



