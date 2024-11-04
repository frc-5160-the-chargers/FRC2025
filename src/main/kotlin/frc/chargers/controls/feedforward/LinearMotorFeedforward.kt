@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Units

/**
 * Represents a feedforward equation that characterizes a
 * linear velocity targeting mechanism with no gravity.
 *
 * This can include flywheels or drivetrain modules.
 *
 * @see SimpleMotorFeedforward
 */
data class LinearMotorFeedforward(
    val kS: Double,
    val kV: Double,
    val kA: Double = 0.0,
    val distanceUnit: Distance = meters
) {
    private val toMetersScalar = (meters / distanceUnit).siValue
    private val baseFF = SimpleMotorFeedforward(kS,kV * toMetersScalar, kA * toMetersScalar)
    private val velocityStore = Units.MetersPerSecond.mutable(0.0)
    private val nextVelocityStore = Units.MetersPerSecond.mutable(0.0)

    operator fun invoke(velocity: Velocity): Voltage {
        velocityStore.mut_replace(velocity.inUnit(meters / seconds), Units.MetersPerSecond)
        return Voltage(baseFF.calculate(velocityStore).baseUnitMagnitude())
    }

    @JvmName("calculatePlantInversion")
    operator fun invoke(
        currentTarget: Velocity,
        nextTarget: Velocity
    ): Voltage {
        if (kV == 0.0 || kS == 0.0) return Voltage(0.0)
        velocityStore.mut_replace(currentTarget.inUnit(meters / seconds), Units.MetersPerSecond)
        nextVelocityStore.mut_replace(nextTarget.inUnit(meters / seconds), Units.MetersPerSecond)
        return Voltage(baseFF.calculate(this.velocityStore, nextVelocityStore).baseUnitMagnitude())
    }

    fun toAngular(gearRatio: Double, wheelRadius: Length): AngularMotorFeedforward =
        AngularMotorFeedforward(
            kS,
            kV * toMetersScalar / gearRatio * wheelRadius.inUnit(meters),
            kA * toMetersScalar / gearRatio * wheelRadius.inUnit(meters),
            radians
        )
}