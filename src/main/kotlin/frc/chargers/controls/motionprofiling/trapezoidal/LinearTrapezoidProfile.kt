@file:Suppress("unused")
package frc.chargers.controls.motionprofiling.trapezoidal

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.motionprofiling.LinearMotionProfile
import frc.chargers.controls.motionprofiling.LinearMotionProfileState

class LinearTrapezoidProfile(
    val maxVelocity: Velocity,
    val maxAcceleration: Acceleration
): LinearMotionProfile {
    private val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity.siValue, maxAcceleration.siValue))

    override fun calculate(
        setpoint: LinearMotionProfileState,
        goal: LinearMotionProfileState,
        dt: Time,
    ): LinearMotionProfileState {
        val profileState = profile.calculate(
            dt.inUnit(seconds),
            setpoint.toTrapezoidalState(),
            goal.toTrapezoidalState()
        )

        return LinearMotionProfileState(
            Distance(profileState.position),
            Velocity(profileState.velocity)
        )
    }
}