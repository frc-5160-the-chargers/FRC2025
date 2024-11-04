@file:Suppress("unused")
package frc.chargers.hardware.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.framework.UnitTesting
import frc.chargers.utils.waitThenRun
import monologue.Annotations.Log
import monologue.Logged


/**
 * A wrapper around the [Pigeon2] class from CTRE,
 * with units support and interface implementation.
 */
class ChargerPigeon2(
    val deviceID: Int,
    canBus: String? = null,
    factoryDefault: Boolean = true,
    headingUpdateFrequency: Frequency? = null,
    var simHeadingSource: () -> Angle = { Angle(0.0) }
): ZeroableHeadingProvider, Logged {
    val base: Pigeon2 = if (canBus == null) Pigeon2(deviceID) else Pigeon2(deviceID, canBus)

    private val yawSignal = base.yaw
    private val rollSignal = base.roll
    private val pitchSignal = base.pitch

    private val yawRateSignal = base.angularVelocityXWorld
    private val pitchRateSignal = base.angularVelocityYWorld
    private val rollRateSignal = base.angularVelocityXWorld

    private val xAccelSignal = base.accelerationX
    private val yAccelSignal = base.accelerationY
    private val zAccelSignal = base.accelerationZ

    private val allSignals = listOf(
        yawSignal, rollSignal, pitchSignal,
        yawRateSignal, pitchRateSignal, rollRateSignal,
        xAccelSignal, yAccelSignal, zAccelSignal
    )
    private var simPreviousYaw = Angle(0.0)

    init {
        UnitTesting.addGlobalCloseable(base)
        waitThenRun(1.seconds) { zeroHeading() }
        if (factoryDefault) base.configurator.apply(Pigeon2Configuration())
        if (headingUpdateFrequency != null) yawSignal.setUpdateFrequency(headingUpdateFrequency.inUnit(hertz))
    }

    /**
     * The heading of the Pigeon; equivalent to yaw.
     */
    override val heading: Angle get() = this.yaw

    /**
     * Zeroes the heading of the Pigeon.
     */
    override fun zeroHeading(angle: Angle) {
        base.setYaw(angle.inUnit(degrees))
    }

    @get:Log(key = "yaw(Rad)")
    val yaw: Angle get() = if (isReal()) yawSignal.refresh().valueAsDouble.ofUnit(degrees) else simHeadingSource()

    @get:Log(key = "pitch(Rad)")
    val pitch: Angle get() = pitchSignal.refresh().valueAsDouble.ofUnit(degrees)

    @get:Log(key = "roll(Rad)")
    val roll: Angle get() = rollSignal.refresh().valueAsDouble.ofUnit(degrees)

    @get:Log(key = "yawRate(Rad/S)")
    val yawRate: AngularVelocity get() = if (isReal()) {
        yawRateSignal.refresh().valueAsDouble.ofUnit(degrees / seconds)
    } else {
        val currH = simHeadingSource()
        ((currH - simPreviousYaw) / 0.02.seconds).also {
            simPreviousYaw = currH
        }
    }

    @get:Log(key = "pitchRate(Rad/S)")
    val pitchRate: AngularVelocity
        get() = pitchRateSignal.refresh().valueAsDouble.ofUnit(degrees / seconds)

    @get:Log(key = "rollRate(Rad/S)")
    val rollRate: AngularVelocity
        get() = rollRateSignal.refresh().valueAsDouble.ofUnit(degrees / seconds)

    @get:Log(key = "Accel(MPS^2)/x")
    val xAcceleration: Acceleration
        get() = xAccelSignal.refresh().valueAsDouble.ofUnit(standardGravities)

    @get:Log(key = "Accel(MPS^2)/x")
    val yAcceleration: Acceleration
        get() = yAccelSignal.refresh().valueAsDouble.ofUnit(standardGravities)

    @get:Log(key = "Accel(MPS^2)/x")
    val zAcceleration: Acceleration
        get() = zAccelSignal.refresh().valueAsDouble.ofUnit(standardGravities)

    /**
     * Determines if the gyro is connected or not.
     */
    @get:Log
    val isConnected get() = allSignals.all { it.status == StatusCode.OK }
}