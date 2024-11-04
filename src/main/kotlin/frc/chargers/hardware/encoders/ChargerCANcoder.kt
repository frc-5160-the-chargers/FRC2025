package frc.chargers.hardware.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.chargers.framework.UnitTesting

/**
 * A wrapper around the [CANcoder] class that implements the [Encoder] interface.
 */
class ChargerCANcoder(
    val deviceID: Int,
    factoryDefault: Boolean = true,
    useAbsolutePosition: Boolean = true,
    sensorDirection: SensorDirectionValue? = null,
    absoluteSensorRange: AbsoluteSensorRangeValue? = null,
    magnetOffset: Angle? = null,
): Encoder {
    /**
     * The base [CANcoder] class.
     */
    val base: CANcoder = CANcoder(deviceID)

    init {
        UnitTesting.addGlobalCloseable(base)
        val config = CANcoderConfiguration()
        if (!factoryDefault) base.configurator.refresh(config)
        if (sensorDirection != null) config.MagnetSensor.SensorDirection = sensorDirection
        if (absoluteSensorRange != null) config.MagnetSensor.AbsoluteSensorRange = absoluteSensorRange
        if (magnetOffset != null) config.MagnetSensor.MagnetOffset = magnetOffset.inUnit(rotations)
        for (i in 1..4) {
            val status = base.configurator.apply(config, 0.1)
            if (status == StatusCode.OK) break
            if (i == 4) Alert("CANcoder($deviceID) failed to configure", AlertType.kError).set(true)
        }
    }

    private val posSignal = if (useAbsolutePosition) base.absolutePosition else base.position
    private val velSignal = base.velocity

    /**
     * Obtains the position of the CANcoder.
     * If useAbsolutePosition is set to false, this measured from the CANcoder's relative encoder;
     * otherwise, it is measured from the CANcoder's absolute encoder.
     */
    override val angularPosition: Angle
        get() = posSignal.refresh(true).valueAsDouble.ofUnit(rotations)

    /**
     * Obtains the velocity of the CANcoder.
     */
    override val angularVelocity: AngularVelocity
        get() = velSignal.refresh(true).valueAsDouble.ofUnit(rotations / seconds)
}