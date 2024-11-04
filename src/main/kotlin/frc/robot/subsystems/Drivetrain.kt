package frc.robot.subsystems

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.grams
import com.batterystaple.kmeasure.units.kilo
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.inches
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.system.plant.DCMotor.getKrakenX60
import edu.wpi.first.math.system.plant.DCMotor.getNEO
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.hardware.motorcontrol.MotorSim
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.Mk4i
import frc.chargers.hardware.subsystems.swervedrive.SwerveConstants

private const val TURN_GEAR_RATIO = Mk4i.TURN_RATIO
private const val DRIVE_GEAR_RATIO = Mk4i.L2_DRIVE_RATIO
private val TURN_MOI = 0.004.ofUnit(kilo.grams * (meters * meters))
private val DRIVE_MOI = 0.025.ofUnit(kilo.grams * (meters * meters))

fun getDrivetrain() =
    EncoderHolonomicDrivetrain(
        turnMotors = List(4) { MotorSim.regular(getNEO(1), TURN_GEAR_RATIO, TURN_MOI) },
        driveMotors = List(4) { MotorSim.regular(getKrakenX60(1), DRIVE_GEAR_RATIO, DRIVE_MOI) },
        constants = SwerveConstants(
            trackWidth = 27.inches,
            wheelBase = 27.inches,
            wheelDiameter = 4.inches,
            azimuthPID = PIDConstants(7.0, 0.0, 0.01),
            velocityPID = PIDConstants(7.0, 0.0, 0.01),
            velocityFF = AngularMotorFeedforward(0.006, 0.13)
        )
    )