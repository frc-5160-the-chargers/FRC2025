@file:Suppress("unused")
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

private const val EPSILON = 1E-9

/** A method that checks if 2 doubles are equal; correcting for floating-point error. */
infix fun Double.epsilonEquals(other: Double): Boolean =
    abs(this - other) < EPSILON

/** A method that checks if 2 quantities are equal; correcting for floating-point error. */
infix fun <D: AnyDimension> Quantity<D>.epsilonEquals(other: Quantity<D>): Boolean =
    siValue epsilonEquals other.siValue