@file:Suppress("unused")
package frc.chargers.hardware.imu

import com.batterystaple.kmeasure.quantities.Angle

/**
 * An interface representing a device capable of calculating the heading of a robot (the direction it is facing).
 */
interface HeadingProvider {
    /**
     * The heading of the robot, with the clockwise direction as positive.
     *
     * Note that this value may be any arbitrary angle, including angles
     * outside the range of [-360º-360º]. This ensures that this value changes
     * continuously, not jumping sharply from 359º to 0º. This also implies
     * that all counterclockwise rotations will increase this value; otherwise,
     * a (say) 2º rotation at position 359º would give 1º, a smaller value
     * due to wrapping around.
     */
    val heading: Angle
}

/**
 * A [HeadingProvider] that can be zeroed.
 */
interface ZeroableHeadingProvider: HeadingProvider {
    /**
     * Zeroes the heading of the [HeadingProvider].
     */
    fun zeroHeading(angle: Angle = Angle(0.0))
}

/**
 * Inline syntax to create a generic [HeadingProvider].
 *
 * I.E:
 * ```
 * val headingProvider = HeadingProvider{ Angle(0.0) }
 */
inline fun HeadingProvider(crossinline getHeading: () -> Angle): HeadingProvider =
    object: HeadingProvider { override val heading: Angle get() = getHeading() }
