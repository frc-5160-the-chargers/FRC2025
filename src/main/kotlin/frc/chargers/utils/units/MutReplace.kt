package frc.chargers.utils.units

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.Units as WPIUnits
import edu.wpi.first.units.measure.MutVoltage

fun MutVoltage.mut_replace(value: Voltage) = mut_replace(value.inUnit(volts), WPIUnits.Volts)
fun MutAngle.mut_replace(value: Angle) = mut_replace(value.inUnit(radians), WPIUnits.Radians)
fun MutDistance.mut_replace(value: Distance) = mut_replace(value.inUnit(meters), WPIUnits.Meters)
fun MutAngularVelocity.mut_replace(value: AngularVelocity) = mut_replace(value.inUnit(radians / seconds), WPIUnits.RadiansPerSecond)
fun MutLinearVelocity.mut_replace(value: Velocity) = mut_replace(value.inUnit(meters / seconds), WPIUnits.MetersPerSecond)
