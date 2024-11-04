@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.units.Units as WPIUnits

/**
 * Represents a feedforward equation that characterizes a
 * linear velocity targeting elevator.
 *
 * @see ElevatorFeedforward
 */
data class UnitElevatorFeedforward(
    val kS: Double,
    val kG: Double,
    val kV: Double,
    val kA: Double = 0.0,
    val distanceUnit: Distance = meters
) {
    private val toMetersScalar = (meters / distanceUnit).siValue
    private val baseFF = ElevatorFeedforward(kS, kG, kV * toMetersScalar, kA * toMetersScalar)
    private val velocityStore = WPIUnits.MetersPerSecond.mutable(0.0)
    private val nextVelocityStore = WPIUnits.MetersPerSecond.mutable(0.0)

    operator fun invoke(velocity: Velocity): Voltage {
        velocityStore.mut_replace(velocity.inUnit(meters / seconds), WPIUnits.MetersPerSecond)
        return Voltage(baseFF.calculate(velocityStore).baseUnitMagnitude())
    }

    @JvmName("calculatePlantInversion")
    operator fun invoke(
        currentTarget: Velocity,
        nextTarget: Velocity
    ): Voltage {
        if (kV == 0.0 || kS == 0.0) return Voltage(0.0)
        velocityStore.mut_replace(currentTarget.inUnit(meters / seconds), WPIUnits.MetersPerSecond)
        nextVelocityStore.mut_replace(nextTarget.inUnit(meters / seconds), WPIUnits.MetersPerSecond)
        return Voltage(baseFF.calculate(this.velocityStore, nextVelocityStore).baseUnitMagnitude())
    }
}