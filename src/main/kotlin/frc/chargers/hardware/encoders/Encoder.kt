@file:Suppress("unused")
package frc.chargers.hardware.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity

/**
 * Represents a generic encoder.
 *
 * An encoder is a device affixed to a motor that measures its rotation.
 * Objects implementing [Encoder] interface support both position and velocity access;
 * use [PositionEncoder] if you only need position values.
 */
interface Encoder : PositionEncoder {
    val angularVelocity: AngularVelocity
}

/**
 * Represents an encoder that can measure the current angular position of a motor.
 */
interface PositionEncoder {
    val angularPosition: Angle

    /**
     * Adds the [offset] to the [angularPosition] of the encoder.
     */
    operator fun plus(offset: Angle): PositionEncoder =
        object: PositionEncoder { override val angularPosition: Angle get() = this@PositionEncoder.angularPosition + offset }

    /**
     * Subtracts the [offset] from the [angularPosition] of the encoder.
     */
    operator fun minus(offset: Angle): PositionEncoder = this + (-offset)
}
