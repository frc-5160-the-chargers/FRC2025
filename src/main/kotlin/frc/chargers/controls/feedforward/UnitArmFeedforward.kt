@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.units.Units
import kotlin.math.cos


/**
 * Represents a feedforward equation that characterizes a
 * linear velocity targeting elevator.
 *
 * @see ArmFeedforward
 */
data class UnitArmFeedforward(
    val kS: Double,
    val kG: Double,
    val kV: Double,
    val kA: Double = 0.0,
    val angleUnit: Angle = radians
) {
    private val toRadiansScalar = (radians / angleUnit).siValue
    private val baseFF = ArmFeedforward(kS, kG * cos(toRadiansScalar), kV * toRadiansScalar, kA * toRadiansScalar)
    private val velocityStore = Units.RadiansPerSecond.mutable(0.0)
    private val nextVelocityStore = Units.RadiansPerSecond.mutable(0.0)
    private val positionStore = Units.Radians.mutable(0.0)

    operator fun invoke(position: Angle, velocity: AngularVelocity): Voltage {
        velocityStore.mut_replace(velocity.inUnit(radians / seconds), Units.RadiansPerSecond)
        positionStore.mut_replace(position.inUnit(radians), Units.Radians)
        return Voltage(baseFF.calculate(positionStore, velocityStore).baseUnitMagnitude())
    }

    @JvmName("calculatePlantInversion")
    operator fun invoke(
        position: Angle,
        velocity: AngularVelocity,
        nextVelocity: AngularVelocity
    ): Voltage {
        velocityStore.mut_replace(velocity.inUnit(radians / seconds), Units.RadiansPerSecond)
        nextVelocityStore.mut_replace(nextVelocity.inUnit(radians / seconds), Units.RadiansPerSecond)
        positionStore.mut_replace(position.inUnit(radians), Units.Radians)
        return Voltage(baseFF.calculate(positionStore, velocityStore, nextVelocityStore).baseUnitMagnitude())
    }
}