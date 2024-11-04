package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.constants
import frc.chargers.hardware.encoders.Encoder
import kotlin.math.PI

class MotorSim(
    linearSystem: LinearSystem<N2, N1, N2>,
    motorType: DCMotor,
    vararg measurementStdDevs: Double
): Motor, SubsystemBase() {
    companion object {
        fun regular(
            motorType: DCMotor, gearRatio: Double, moi: MomentOfInertia,
            vararg measurementStdDevs: Double
        ) = MotorSim(
            LinearSystemId.createDCMotorSystem(motorType, moi.inUnit(kilo.grams * (meters * meters)), gearRatio),
            motorType, *measurementStdDevs
        )

        fun arm(
            motorType: DCMotor, gearRatio: Double, moi: MomentOfInertia,
            vararg measurementStdDevs: Double
        ) = MotorSim(
            LinearSystemId.createSingleJointedArmSystem(motorType, moi.inUnit(kilo.grams * (meters * meters)), gearRatio),
            motorType, *measurementStdDevs
        )

        fun elevator(
            motorType: DCMotor, gearRatio: Double, mass: Mass, moi: MomentOfInertia,
            vararg measurementStdDevs: Double
        ) = MotorSim(
            LinearSystemId.createElevatorSystem(
                motorType, mass.inUnit(kilo.grams),
                moi.inUnit(kilo.grams * (meters * meters)), gearRatio
            ),
            motorType, *measurementStdDevs
        )
    }

    private var base = DCMotorSim(linearSystem, motorType, *measurementStdDevs)
    private val positionController = PIDController(0.0, 0.0, 0.0)
    private val velocityController = PIDController(0.0, 0.0, 0.0)

    private var positionPIDConfigured = false
    private var velocityPIDConfigured = false

    override fun simulationPeriodic() {
        base.update(0.02)
    }

    override val encoder: Encoder = SimEncoder()
    private inner class SimEncoder : Encoder {
        override val angularPosition: Angle
            get() = base.angularPositionRad.ofUnit(radians)

        override val angularVelocity: AngularVelocity
            get() = base.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override var voltageOut: Voltage = 0.volts
        set(value) {
            field = value
            base.inputVoltage = value.siValue * if (inverted) -1.0 else 1.0
        }

    override val statorCurrent: Current
        get() = base.currentDrawAmps.ofUnit(amps)

    override var inverted = false

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        require(positionPIDConfigured){" You must specify a positionPID value using the configure(positionPID = PIDConstants(p,i,d)) method. "}
        var encoderReading = this.encoder.angularPosition
        if (positionController.isContinuousInputEnabled) encoderReading %= 360.degrees
        val pidOutput = positionController.calculate(encoderReading.inUnit(radians), position.inUnit(radians))
        this.voltageOut = Voltage(pidOutput) + feedforward
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        require(velocityPIDConfigured){" You must specify a velocityPID value using the configure(velocityPID = PIDConstants(p,i,d)) method. "}
        val pidOutput = velocityController.calculate(encoder.angularVelocity.siValue, velocity.siValue)
        this.voltageOut = Voltage(pidOutput) + feedforward
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
    ): MotorSim {
        if (inverted != null) this.inverted = inverted
        require(followerMotors.isEmpty()) { "Don't add followers to a simulated motor; consider increasing the # of motors in the DCMotor.getNEO or DCMotor.getNeoVortex calls." }
        if (positionPID != null) positionController.constants = positionPID; positionPIDConfigured = true
        if (velocityPID != null) velocityController.constants = velocityPID; velocityPIDConfigured = true
        if (continuousInput == true) {
            positionController.enableContinuousInput(0.0, 2 * PI)
        } else if (continuousInput == false) {
            positionController.disableContinuousInput()
        }
        return this
    }
}