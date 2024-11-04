@file:Suppress("unused")
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator

/**
 * A variant of [InterpolatingTreeMap] but with Kmeasure [Quantity]s instead.
 *
 * ```
 * val map = InterpolatingQuantityTreeMap(
 *     0.meters to 90.degrees,
 *     5.meters to 60.degrees
 * )
 *
 * val output = map[2.5.meters]
 * map[2.5.meters] = someValue
 *
 * ```
 */
class InterpolatingQuantityTreeMap<K: AnyDimension, V: AnyDimension>():
    InterpolatingTreeMap<Quantity<K>, Quantity<V>>(getInverseInterpolator(), getInterpolator()) {

    constructor(vararg pairs: Pair<Quantity<K>, Quantity<V>>): this() {
        for ((key, value) in pairs){
            this.put(key, value)
        }
    }

    /**
     * Allows for the full use of array access syntax for interpolating maps.
     *
     * Note: the get method of the map is already transformed into an operator method.
     */
    operator fun set(key: Quantity<K>, value: Quantity<V>) {
        this.put(key, value)
    }
}

private fun <K: AnyDimension> getInverseInterpolator() =
    InverseInterpolator<Quantity<K>>{ start, end, q ->
        MathUtil.inverseInterpolate(start.siValue, end.siValue, q.siValue)
    }

private fun <V: AnyDimension> getInterpolator() =
    Interpolator<Quantity<V>>{ start, end, q ->
        Quantity(MathUtil.interpolate(start.siValue, end.siValue, q))
    }