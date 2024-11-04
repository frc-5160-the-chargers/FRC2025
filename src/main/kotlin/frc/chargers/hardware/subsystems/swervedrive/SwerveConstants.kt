package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.controls.motionprofiling.AngularMotionProfile

private val DEFAULT_MAX_MODULE_SPEED = 4.5.ofUnit(meters / seconds)

object Mk4i {
    const val L2_DRIVE_RATIO = 6.75
    const val L3_DRIVE_RATIO = 6.12
    const val TURN_RATIO = 150.0 / 7.0
    val WHEEL_DIAMETER = 4.inches
}

data class SwerveConstants(
    val trackWidth: Distance,
    val wheelBase: Distance,
    val wheelDiameter: Distance,
    val robotRotationPID: PIDConstants = PIDConstants(6.0,0.0,0.0), // rotation of the entire robot; for pathplanner
    val robotTranslationPID: PIDConstants = PIDConstants(4.0,0.0,0.0), // translation of the entire robot; for pathplanner
    val couplingRatio: Double? = null,
    val driveMotorMaxSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,

    val azimuthPID: PIDConstants,
    val azimuthPIDTolerance: Angle? = null,
    val azimuthMotionProfile: AngularMotionProfile? = null,
    val azimuthFF: AngularMotorFeedforward = AngularMotorFeedforward(0.0, 0.0, 0.0),

    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFeedforward,
)