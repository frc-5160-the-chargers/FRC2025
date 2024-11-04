@file:Suppress("unused")
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import java.util.*

/*
Credits go to 6328 & 4099 for all of the code below;
some edits were applied to better suit kotlin(such as swapping to extension functions).
 */

private val FIELD_LENGTH = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().fieldLength
private fun shouldFlip(): Boolean = DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)

/** Flips a blue alliance pose to the correct side of the field based on the current alliance color. */
fun Pose2d.flipWhenRedAlliance(): Pose2d =
    if (shouldFlip()) {
        Pose2d(FIELD_LENGTH - x, y, Rotation2d(-rotation.cos, rotation.sin))
    } else {
        this
    }

/** Flips a blue alliance translation to the correct side of the field based on the current alliance color. */
fun Translation2d.flipWhenRedAlliance(): Translation2d =
    if (shouldFlip()) {
        Translation2d(FIELD_LENGTH - x, y)
    } else {
        this
    }

/** Flips a rotation based on the current alliance color. */
fun Angle.flipWhenRedAlliance(): Angle =
    if (shouldFlip()) {
        Rotation2d(-cos(this), sin(this)).angle
    } else {
        this
    }

/** Flips an x coordinate to the correct side of the field based on the current alliance color. */
fun flipXCoordinateWhenRedAlliance(xCoordinate: Length): Length =
    if (shouldFlip()) {
        FIELD_LENGTH.ofUnit(meters) - xCoordinate
    } else {
        xCoordinate
    }

