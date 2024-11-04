package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import com.revrobotics.*
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.chargers.framework.UnitTesting
import frc.chargers.hardware.encoders.Encoder
import frc.chargers.utils.units.frequencyToPeriod
import kotlin.math.PI
import kotlin.math.roundToInt


/**
 * A [CANSparkMax] motor controller that implements the [Motor] interface
 * and performs periodic self-checking.
 *
 * The [base] property allows for the access of the base [CANSparkMax] instance.
 */
class ChargerSparkMax(
    deviceID: Int,
    gearRatio: Double,
    motorType: CANSparkLowLevel.MotorType = CANSparkLowLevel.MotorType.kBrushless,
    useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true,
    faultLogName: String? = null
): ChargerSpark<CANSparkMax>(
    CANSparkMax(deviceID, motorType),
    gearRatio, useAbsoluteEncoder, factoryDefault, faultLogName
)


/**
 * A [CANSparkFlex] motor controller that implements the [Motor] interface
 * and performs periodic self-checking.
 *
 * The [base] property allows for the access of the base [CANSparkFlex] instance.
 */
class ChargerSparkFlex(
    deviceID: Int,
    gearRatio: Double,
    useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true,
    faultLogName: String? = null
): ChargerSpark<CANSparkFlex>(
    CANSparkFlex(deviceID, CANSparkLowLevel.MotorType.kBrushless),
    gearRatio, useAbsoluteEncoder, factoryDefault, faultLogName
)


open class ChargerSpark<BaseMotorType: CANSparkBase>(
    /**
     * The base [CANSparkFlex]/[CANSparkMax] instance.
     */
    val base: BaseMotorType,
    gearRatio: Double,
    private val useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true,
    private val faultLogName: String? = null
): Motor {
    val deviceID: Int = base.deviceId

    private val nonRevFollowers = mutableListOf<Motor>()
    private val relativeEncoder = base.encoder
    private val absoluteEncoder = base.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private var positionPIDConfigured = false
    private var velocityPIDConfigured = false

    private val errors = mutableListOf<REVLibError>()
    private fun REVLibError.bind() { if (this != REVLibError.kOk) errors.add(this) }
    private val faultAlert = Alert("Spark Max Error", "", AlertType.kError)

    init {
        UnitTesting.addGlobalCloseable(base)
        for (i in 1..4) {
            errors.clear()
            if (factoryDefault) base.restoreFactoryDefaults().bind()
            if (useAbsoluteEncoder) {
                base.pidController.setFeedbackDevice(absoluteEncoder).bind()
                absoluteEncoder.setPositionConversionFactor(1 / gearRatio).bind()
                absoluteEncoder.setVelocityConversionFactor(1 / gearRatio).bind()
            } else {
                base.pidController.setFeedbackDevice(relativeEncoder).bind()
                relativeEncoder.setPositionConversionFactor(1 / gearRatio).bind()
                relativeEncoder.setVelocityConversionFactor(1 / gearRatio).bind()
            }
            base.burnFlash().bind()
            if (errors.isEmpty()) break
        }
        require(errors.isEmpty()) { "Gear Ratio of ChargerSparkMax(id=$deviceID) not configured; critical error." }
    }

    override fun logFaults() {
        val err = base.lastError
        if (err != REVLibError.kOk) {
            faultAlert.setText(err.toString())
            faultAlert.set(true)
        }
    }

    override val encoder: Encoder = if (useAbsoluteEncoder) AbsoluteEncoderImpl() else RelativeEncoderImpl()
    private inner class RelativeEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity
            get() = relativeEncoder.velocity.ofUnit(rotations / minutes)

        override val angularPosition: Angle
            get() = relativeEncoder.position.ofUnit(rotations)
    }
    private inner class AbsoluteEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity
            get() = absoluteEncoder.velocity.ofUnit(rotations / seconds)

        override val angularPosition: Angle
            get() = absoluteEncoder.position.ofUnit(rotations)
    }

    override var voltageOut: Voltage
        get() = (base.appliedOutput * base.busVoltage).ofUnit(volts)
        set(voltage){
            base.setVoltage(voltage.inUnit(volts))
        }

    override val statorCurrent get() = base.outputCurrent.ofUnit(amps)

    override val inverted get() = base.inverted

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        if (!positionPIDConfigured) {
            Motor.alertVelocityPIDErr()
            return
        } else if (abs(position - this.encoder.angularPosition) < 1.degrees) {
            base.setVoltage(0.0)
            return
        }
        base.pidController.setReference(
            position.inUnit(rotations),
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.inUnit(volts)
        )
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        if (!velocityPIDConfigured) {
            Motor.alertVelocityPIDErr()
            return
        }
        base.pidController.setReference(
            if (useAbsoluteEncoder) {
                velocity.inUnit(rotations / seconds)
            } else {
                velocity.inUnit(rotations / minutes)
            },
            CANSparkBase.ControlType.kVelocity,
            1,
            feedforward.inUnit(volts)
        )
    }

    override fun configure(
        inverted: Boolean?,
        brakeWhenIdle: Boolean?,
        rampRate: Time?,
        statorCurrentLimit: Current?,
        followerMotors: List<Motor>,
        positionUpdateRate: Frequency?,
        velocityUpdateRate: Frequency?,
        optimizeUpdateRate: Boolean?,
        currentPosition: Angle?,
        positionPID: PIDConstants?,
        velocityPID: PIDConstants?,
        continuousInput: Boolean?
    ): ChargerSpark<BaseMotorType> {
        if (inverted != null) base.inverted = inverted
        for (i in 1..4) {
            errors.clear()
            when (brakeWhenIdle) {
                true -> base.setIdleMode(CANSparkBase.IdleMode.kBrake).bind()
                false -> base.setIdleMode(CANSparkBase.IdleMode.kCoast).bind()
                null -> {}
            }
            if (rampRate != null) {
                base.setOpenLoopRampRate(rampRate.inUnit(seconds)).bind()
                base.setClosedLoopRampRate(rampRate.inUnit(seconds)).bind()
            }
            if (statorCurrentLimit != null) base.setSmartCurrentLimit(statorCurrent.inUnit(amps).roundToInt()).bind()
            for (follower in followerMotors) {
                follower.configure(
                    positionPID = positionPID,
                    velocityPID = velocityPID,
                    currentPosition = currentPosition
                )
                when (follower) {
                    is ChargerSpark<*> -> follower.base.follow(base, follower.inverted).bind()
                    else -> nonRevFollowers.add(follower)
                }
            }
            if (currentPosition != null) relativeEncoder.setPosition(currentPosition.inUnit(rotations)).bind()
            // 2 * PI makes it so that the PID gains are optimized off of radians and not rotations
            if (positionPID != null) {
                positionPIDConfigured = true
                base.pidController.apply {
                    setP(positionPID.kP * (2 * PI), 0).bind()
                    setI(positionPID.kI * (2 * PI), 0).bind()
                    setD(positionPID.kD * (2 * PI), 0).bind()
                }
            }
            if (velocityPID != null) {
                velocityPIDConfigured = true
                // relative encoder pid is based off of rotations/min so we have to compensate here
                val multiplier = 2 * PI * if (useAbsoluteEncoder) 1.0 else 60.0
                base.pidController.apply {
                    setP(velocityPID.kP * multiplier, 1).bind()
                    setI(velocityPID.kI * multiplier, 1).bind()
                    setD(velocityPID.kD * multiplier, 1).bind()
                }
            }
            if (continuousInput == true) {
                base.pidController.apply {
                    setPositionPIDWrappingEnabled(true).bind()
                    setPositionPIDWrappingMinInput(-180.degrees.inUnit(rotations)).bind()
                    setPositionPIDWrappingMaxInput(180.degrees.inUnit(rotations)).bind()
                }
            } else if (continuousInput == false) {
                base.pidController.setPositionPIDWrappingEnabled(false).bind()
            }
            if (optimizeUpdateRate == true) {
                val disabledFrames = if (useAbsoluteEncoder) {
                    listOf(PeriodicFrame.kStatus2, PeriodicFrame.kStatus3, PeriodicFrame.kStatus4)
                } else {
                    listOf(PeriodicFrame.kStatus3, PeriodicFrame.kStatus4, PeriodicFrame.kStatus5, PeriodicFrame.kStatus6)
                }
                for (frame in disabledFrames) {
                    base.setPeriodicFramePeriod(frame, 65535)
                }
            }
            if (positionUpdateRate != null) {
                base.setPeriodicFramePeriod(
                    if (useAbsoluteEncoder) PeriodicFrame.kStatus5 else PeriodicFrame.kStatus2,
                    frequencyToPeriod(positionUpdateRate).inUnit(milli.seconds).roundToInt()
                ).bind()
            }
            if (velocityUpdateRate != null) {
                base.setPeriodicFramePeriod(
                    if (useAbsoluteEncoder) PeriodicFrame.kStatus6 else PeriodicFrame.kStatus1,
                    frequencyToPeriod(velocityUpdateRate).inUnit(milli.seconds).roundToInt()
                ).bind()
            }
            base.burnFlash().bind()
            if (errors.isEmpty()) return this
        }
        Alert(
            "${faultLogName ?: "ChargerSpark($deviceID)"} could not configure. Errors: $errors",
            AlertType.kError
        ).set(true)
        return this
    }
}