package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.RobotController
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.encoders.PositionEncoder
import frc.chargers.utils.units.periodToFrequency
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.angle
import monologue.Logged
import kotlin.math.abs
import kotlin.math.cos


class SwerveModule(
    private val turnMotor: Motor,
    // turn encoders are optional in sim
    private val turnEncoder: PositionEncoder? = null,
    private val driveMotor: Motor,
    private val constants: SwerveConstants
): Logged {
    private val startingDirection: Angle? = if (turnEncoder != null) turnEncoder.angularPosition % 360.degrees else null
    private val wheelRadius = constants.wheelDiameter / 2
    private var couplingOffset = 0.degrees
    private var azimuthProfileState = AngularMotionProfileState(startingDirection ?: turnMotor.encoder.angularPosition)

    // instead of using the abs encoder reading directly,
    // we configure the motor's gear ratio and starting position off the abs encoder
    val direction get() = (turnMotor.encoder.angularPosition) % 360.degrees

    val driveAngularVelocity get() = driveMotor.encoder.angularVelocity

    val driveLinearVelocity get() = driveAngularVelocity * wheelRadius

    val wheelTravel get() = (driveMotor.encoder.angularPosition - couplingOffset) * wheelRadius

    init {
        turnMotor.configure(
            currentPosition = startingDirection,
            positionPID = constants.azimuthPID,
            continuousInput = true,
            positionUpdateRate = periodToFrequency(constants.odometryUpdateRate)
        )
        driveMotor.configure(
            currentPosition = 0.degrees,
            velocityPID = constants.velocityPID,
            positionUpdateRate = periodToFrequency(constants.odometryUpdateRate)
        )
        log("UsingCouplingRatio", constants.couplingRatio != null)
    }

    fun periodic() {
        if (constants.couplingRatio != null){
            couplingOffset -= constants.couplingRatio * (direction - 180.degrees)
            log("CouplingOffset(Deg)", couplingOffset.inUnit(degrees))
        }
        if (turnEncoder != null) log("AbsoluteEncoderReading(Deg)", turnEncoder.angularPosition.inUnit(degrees))
        log("Direction(Deg)", direction.inUnit(degrees))
        log("DriveVel(MPS)", driveLinearVelocity.inUnit(meters / seconds))
        log("WheelTravel(M)", wheelTravel.inUnit(meters))
        log("TurnMotorCurrent", turnMotor.statorCurrent.inUnit(amps))
        log("DriveMotorCurrent", driveMotor.statorCurrent.inUnit(amps))
        log("TurnVel(DegPerSec)", turnMotor.encoder.angularVelocity.inUnit(degrees / seconds))
    }

    fun setUpdateRate(period: Frequency) {
        driveMotor.configure(velocityUpdateRate = period)
        turnMotor.configure(positionUpdateRate = period)
    }

    fun syncTurnEncoder() {
        if (turnEncoder != null) turnMotor.configure(currentPosition = turnEncoder.angularPosition)
    }

    fun getModuleState() = SwerveModuleState(
        driveLinearVelocity.inUnit(meters / seconds),
        Rotation2d(direction)
    )

    fun getModulePosition() = SwerveModulePosition(
        wheelTravel.inUnit(meters),
        Rotation2d(direction)
    )

    fun setDriveVoltage(target: Voltage) {
        driveMotor.voltageOut = target
        log("DriveVoltage", target.inUnit(volts))
    }

    fun setTurnVoltage(target: Voltage) {
        val trueVoltage = if (abs(target) < 0.01.volts) 0.volts else target
        turnMotor.voltageOut = trueVoltage
        log("TurnVoltage", trueVoltage.inUnit(volts))
    }

    private fun calculateSetpoint(motionProfile: AngularMotionProfile, goalState: AngularMotionProfileState) =
        motionProfile.calculate(
            setpoint = azimuthProfileState,
            goal = goalState,
            measurement = direction,
            continuousInputRange = 0.degrees..360.degrees
        )

    fun setDirection(target: Angle) {
        // utilizes custom absolute value overload for kmeasure quantities
        if (constants.azimuthPIDTolerance != null &&
            abs(direction - target) < constants.azimuthPIDTolerance){
            setTurnVoltage(0.volts)
            azimuthProfileState.position = direction
            azimuthProfileState.velocity = AngularVelocity(0.0)
            return
        }

        val pidTarget: Angle
        val feedforwardV: Voltage

        if (constants.azimuthMotionProfile != null) {
            val goalState = AngularMotionProfileState(target)
            // increments the setpoint by calculating a new one
            azimuthProfileState = calculateSetpoint(constants.azimuthMotionProfile, goalState)
            // Calculates the setpoint 1 loop period in the future,
            // in order to do plant inversion feedforward.
            val futureSetpoint = calculateSetpoint(constants.azimuthMotionProfile, goalState)
            feedforwardV = constants.azimuthFF(
                azimuthProfileState.velocity,
                futureSetpoint.velocity
            )
            pidTarget = azimuthProfileState.position
        } else {
            pidTarget = target
            feedforwardV = 0.volts
        }

        // uses pid control to move to the calculated setpoint, to achieve the target goal.
        turnMotor.setPositionSetpoint(pidTarget, feedforwardV)
    }

    fun setDesiredStateOpenLoop(state: SwerveModuleState) {
        state.optimize(Rotation2d(direction))
        state.speedMetersPerSecond *= abs(
            cos(state.angle.getRadians() - direction.inUnit(radians))
        )
        setDirection(state.angle.angle)
        setDriveVoltage(
            (state.speedMetersPerSecond / constants.driveMotorMaxSpeed.inUnit(meters / seconds) * 12.0)
                .coerceIn(getVoltageRange())
                .ofUnit(volts)
        )
    }

    fun setDesiredStateClosedLoop(state: SwerveModuleState) {
        state.optimize(Rotation2d(direction))
        setDirection(state.angle.angle)
        val velocitySetpoint = state.speedMetersPerSecond.ofUnit(meters / seconds) / wheelRadius
        val feedforwardV = constants.velocityFF(velocitySetpoint)
        driveMotor.setVelocitySetpoint(velocitySetpoint, feedforwardV)
    }

    private val batteryVoltageAlert = Alert("RobotController battery voltage is extremely low(<1volts).", AlertType.kWarning)
    private fun getVoltageRange(): ClosedRange<Double>{
        val upperLimit = RobotController.getBatteryVoltage()
        batteryVoltageAlert.set(upperLimit < 1.0)
        return -upperLimit..upperLimit
    }
}