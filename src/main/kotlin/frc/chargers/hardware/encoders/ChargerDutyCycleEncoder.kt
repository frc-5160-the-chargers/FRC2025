@file:Suppress("unused")
package frc.chargers.hardware.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.rotations
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.chargers.framework.UnitTesting

/**
 * An Adapter of WPILib's [DutyCycleEncoder] class; consists of REV through bore encoders and CTRE mag encoders.
 */
class ChargerDutyCycleEncoder(
    val channel: Int,
    var inverted: Boolean = false
): PositionEncoder {
    val base: DutyCycleEncoder = DutyCycleEncoder(channel)

    init { UnitTesting.addGlobalCloseable(base) }

    override val angularPosition: Angle
        get() = (if (inverted) -1.0 else 1.0) * base.get().ofUnit(rotations)
}