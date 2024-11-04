@file:Suppress("unused")
package frc.chargers.hardware.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.utils.waitThenRun
import monologue.Annotations.Log
import monologue.Logged


class ChargerNavX(
    val base: AHRS = AHRS(),
    private val useFusedHeading: Boolean = false,
    var simHeadingSource: () -> Angle = { Angle(0.0) }
): ZeroableHeadingProvider, Logged {
    private var headingOffset = 0.degrees

    init {
        waitThenRun(0.1.seconds) { zeroHeading() }
    }

    /**
     * Equivalent to the NavX [yaw].
     */
    override val heading get() = this.yaw

    /**
     * Zeroes the heading of the NavX.
     *
     * Does not zero the yaw; call navX.gyroscope.zeroYaw() if this is desired instead.
     */
    override fun zeroHeading(angle: Angle){
        if (isReal()){
            while (base.isCalibrating){
                println("Waiting for AHRS to calibrate...")
                // Wait for 1 second (hardware initialization) before zeroing heading
                Thread.sleep(1000)
            }

            println("AHRS has finished calibrating.")

            headingOffset = -heading + headingOffset + angle

            println("AHRS angle and fused heading has been offset. This does not impact ahrs.getYaw()!")
        }else{
            headingOffset = -simHeadingSource()
            println("AHRS heading has been offset in sim.")
        }
    }

    /**
     * The NavX firmware version.
     */
    @Log val firmwareVersion: String = base.firmwareVersion

    /**
     * Returns if the NavX is connected or not. Automatically false during simulation.
     */
    @get:Log(key = "connected")
    val isConnected get() = base.isConnected && isReal()

    /**
     * The heading of the NavX. Reports a continuous value(can go over 360 and -360 degrees).
     */
    @get:Log(key = "yaw(Rad)")
    val yaw get() = if (isReal()) {
        // Negative sign because the navX reports clockwise as positive, whereas we want counterclockwise to be positive
        if (useFusedHeading && base.isMagnetometerCalibrated && !base.isMagneticDisturbance){
            -base.fusedHeading.toDouble().ofUnit(degrees)
        }else{
            -base.angle.ofUnit(degrees)
        }
    }else{
        simHeadingSource()
    } + headingOffset

    /**
     * The NavX pitch. Is between -180 and 180 degrees.
     */
    @get:Log(key = "pitch(Rad)")
    val pitch get() = base.pitch.toDouble().ofUnit(degrees)

    /**
     * The NavX roll. Is between -180 and 180 degrees.
     */
    @get:Log(key = "roll(Rad)")
    val roll get() = base.roll.toDouble().ofUnit(degrees)

    /**
     * The angular acceleration of the robot.
     */
    @get:Log(key = "yawRate(RadPerSec)")
    val yawRate get() = base.rate.ofUnit(degrees / seconds)

    @get:Log(key = "Accel(MetersPerSec^2)/x")
    val xAcceleration: Acceleration
        get() = base.worldLinearAccelX.toDouble().ofUnit(standardGravities)

    @get:Log(key = "Accel(MetersPerSec^2)/y")
    val yAcceleration: Acceleration
        get() = base.worldLinearAccelY.toDouble().ofUnit(standardGravities)

    @get:Log(key = "Accel(MetersPerSec^2)/z")
    val zAcceleration: Acceleration
        get() = base.worldLinearAccelZ.toDouble().ofUnit(standardGravities)
}