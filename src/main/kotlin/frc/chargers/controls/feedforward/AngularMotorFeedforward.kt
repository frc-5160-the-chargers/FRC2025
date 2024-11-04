@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Units


/**
 * Represents a feedforward equation that characterizes an
 * angular velocity targeting mechanism with no gravity.
 *
 * This can include flywheels or drivetrain modules.
 *
 * @see SimpleMotorFeedforward
 */
data class AngularMotorFeedforward(
    val kS: Double,
    val kV: Double,
    val kA: Double = 0.0,
    val angleUnit: Angle = radians
) {
    private val toRadiansScalar = (radians / angleUnit).siValue
    private val baseFF = SimpleMotorFeedforward(kS, kV * toRadiansScalar, kA * toRadiansScalar)
    private val velocityStore = Units.RadiansPerSecond.mutable(0.0)
    private val nextVelocityStore = Units.RadiansPerSecond.mutable(0.0)

    operator fun invoke(velocity: AngularVelocity): Voltage {
        velocityStore.mut_replace(velocity.inUnit(radians / seconds), Units.RadiansPerSecond)
        return Voltage(baseFF.calculate(velocityStore).baseUnitMagnitude())
    }

    @JvmName("calculatePlantInversion")
    operator fun invoke(velocity: AngularVelocity, nextVelocity: AngularVelocity): Voltage {
        velocityStore.mut_replace(velocity.inUnit(radians / seconds), Units.RadiansPerSecond)
        nextVelocityStore.mut_replace(nextVelocity.inUnit(radians / seconds), Units.RadiansPerSecond)
        return Voltage(baseFF.calculate(velocityStore, nextVelocityStore).baseUnitMagnitude())
    }

    fun toLinear(gearRatio: Double, wheelRadius: Length): LinearMotorFeedforward =
        LinearMotorFeedforward(
            kS,
            kV * toRadiansScalar * gearRatio / wheelRadius.inUnit(meters),
            kA * toRadiansScalar * gearRatio / wheelRadius.inUnit(meters),
            meters
        )
}