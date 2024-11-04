package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile

typealias AngularMotionProfile = MotionProfile<AngleDimension, AngularVelocityDimension>
typealias AngularMotionProfileState = MotionProfileState<AngleDimension, AngularVelocityDimension>

typealias LinearMotionProfile = MotionProfile<DistanceDimension, VelocityDimension>
typealias LinearMotionProfileState = MotionProfileState<DistanceDimension, VelocityDimension>

/**
 * Represents a Generic motion profile, that can calculate an intermediate state
 * between a goal and a setpoint, acheiving smoother and more optimal motion.
 *
 * This is most commonly paired with a controller of some kind; like PID or LQR.
 *
 * This interface has an identical API to WPILib's TrapezoidProfile and ExponentialProfile;
 * however, the "t" argument is moved to the end of function calls(and is now optional).
 */
interface MotionProfile<Pos: AnyDimension, Vel: AnyDimension> {
    fun calculate(
        setpoint: MotionProfileState<Pos, Vel>,
        goal: MotionProfileState<Pos, Vel>,
        dt: Time = 0.02.seconds
    ): MotionProfileState<Pos,Vel>

    fun calculate(
        setpoint: MotionProfileState<Pos, Vel>,
        goal: MotionProfileState<Pos, Vel>,
        measurement: Quantity<Pos>,
        continuousInputRange: ClosedRange<Quantity<Pos>>,
        dt: Time = 0.02.seconds
    ): MotionProfileState<Pos,Vel> {
        val errorBound = (continuousInputRange.endInclusive - continuousInputRange.start) / 2.0

        val goalMinDistance = (goal.position - measurement) % (2 * errorBound) - errorBound
        val setpointMinDistance = (setpoint.position - measurement) % (2 * errorBound) - errorBound

        goal.position = goalMinDistance + measurement
        setpoint.position = setpointMinDistance + measurement

        return calculate(setpoint, goal, dt)
    }
}


data class MotionProfileState <Pos: AnyDimension, Vel: AnyDimension>(
    var position: Quantity<Pos> = Quantity(0.0),
    var velocity: Quantity<Vel> = Quantity(0.0)
){
    fun toTrapezoidalState(): TrapezoidProfile.State =
        TrapezoidProfile.State(position.siValue, velocity.siValue)

    fun toExponentialState(): ExponentialProfile.State =
        ExponentialProfile.State(position.siValue, velocity.siValue)
}

