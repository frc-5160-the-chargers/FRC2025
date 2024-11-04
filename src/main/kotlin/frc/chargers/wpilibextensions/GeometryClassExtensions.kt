@file:Suppress("unused")
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.geometry.*
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt

/*
The functions below are overloads of the WPILib geometry classes,
which utilize kmeasure quantities instead of bare Doubles.

There are also useful conversion functions as well; such as Rotation2d.angle(which converts a Rotation2d to an angle).
 */

fun Rotation2d(angle: Angle): Rotation2d =
    Rotation2d.fromRadians(angle.inUnit(radians))

val Rotation2d.angle: Angle get() = this.getRadians().ofUnit(com.batterystaple.kmeasure.units.radians)


fun Pose2d(x: Distance, y: Distance, rotation: Angle = Angle(0.0)): Pose2d =
    Pose2d(x.inUnit(meters), y.inUnit(meters), Rotation2d(rotation))

fun Pose2d.distanceTo(other: Pose2d): Distance =
    hypot(this.x - other.x, this.y - other.y).ofUnit(meters)


fun Transform2d(x: Distance, y: Distance, rotation: Angle = Angle(0.0)): Transform2d =
    Transform2d(x.inUnit(meters), y.inUnit(meters), Rotation2d(rotation))


@JvmName("Translation2dWithXAndY")
fun Translation2d(x: Distance, y: Distance): Translation2d =
    Translation2d(x.inUnit(meters), y.inUnit(meters))

@JvmName("Translation2dWithNormAndAngle")
fun Translation2d(norm: Distance, angle: Angle): Translation2d =
    Translation2d(norm.inUnit(meters), Rotation2d(angle))





fun Rotation3d(roll: Angle, pitch: Angle, yaw: Angle): Rotation3d = Rotation3d(
    roll.inUnit(radians),
    pitch.inUnit(radians),
    yaw.inUnit(radians)
)

val Rotation3d.roll: Angle get() = this.x.ofUnit(radians)

val Rotation3d.pitch: Angle get() = this.y.ofUnit(radians)

val Rotation3d.yaw: Angle get() = this.z.ofUnit(radians)


fun Translation3d(x: Distance, y: Distance, z: Distance): Translation3d =
    Translation3d(x.inUnit(meters), y.inUnit(meters), z.inUnit(meters))


fun Transform3d(x: Distance, y: Distance, z: Distance, rotation: Rotation3d): Transform3d =
    Transform3d(x.inUnit(meters), y.inUnit(meters), z.inUnit(meters), rotation)


fun Pose3d(x: Distance, y: Distance, z: Distance, rotation: Rotation3d): Pose3d =
    Pose3d(x.inUnit(meters), y.inUnit(meters), z.inUnit(meters), rotation)

fun Pose3d.distanceTo(other: Pose3d): Distance =
    sqrt(
        (this.x - other.x).pow(2) +
        (this.y - other.y).pow(2) +
        (this.z - other.z).pow(2)
    ).ofUnit(meters)
