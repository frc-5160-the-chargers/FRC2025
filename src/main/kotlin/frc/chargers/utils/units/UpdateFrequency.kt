@file:Suppress("unused")
package frc.chargers.utils.units

import com.batterystaple.kmeasure.quantities.Frequency
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.seconds

fun periodToFrequency(period: Time): Frequency =
    (1.0 / period.inUnit(seconds)).ofUnit(hertz)

fun frequencyToPeriod(frequency: Frequency): Time =
    (1.0 / frequency.inUnit(hertz)).ofUnit(seconds)