@file:Suppress("unused")
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.ChassisSpeeds


/**
 * A convenience function that creates a [ChassisSpeeds]
 * with kmeasure Quantities.
 */
fun ChassisSpeeds(xVelocity: Velocity, yVelocity: Velocity, rotationalVelocity: AngularVelocity): ChassisSpeeds =
    ChassisSpeeds(
        xVelocity.inUnit(meters/seconds),
        yVelocity.inUnit(meters/seconds),
        rotationalVelocity.inUnit(radians/seconds)
    )

val ChassisSpeeds.xVelocity: Velocity
    get() = vxMetersPerSecond.ofUnit(meters/seconds)

val ChassisSpeeds.yVelocity: Velocity
    get() = vyMetersPerSecond.ofUnit(meters/seconds)

val ChassisSpeeds.rotationalVelocity: AngularVelocity
    get() = omegaRadiansPerSecond.ofUnit(radians/seconds)







