// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.chargers.wpilibextensions.kinematics

import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import java.nio.ByteBuffer
import kotlin.math.abs

/**
 * A helper class that stores direction powers for drivetrain classes.
 */
data class ChassisPowers(
    var xPower: Double = 0.0,
    var yPower: Double = 0.0,
    var rotationPower: Double = 0.0
): StructSerializable {
    companion object {
        @JvmField
        val struct = object: Struct<ChassisPowers> {
            override fun getTypeClass(): Class<ChassisPowers> = ChassisPowers::class.java
            override fun getTypeName(): String = "ChassisPowers"
            override fun getTypeString(): String = "struct:ChassisPowers"
            override fun getSize(): Int = Struct.kSizeDouble * 3
            override fun getSchema(): String = "double xPower;double yPower;double rotationPower;"
            override fun unpack(bb: ByteBuffer): ChassisPowers =
                ChassisPowers(bb.getDouble(), bb.getDouble(), bb.getDouble())
            override fun pack(bb: ByteBuffer, value: ChassisPowers) {
                bb.putDouble(value.xPower)
                bb.putDouble(value.yPower)
                bb.putDouble(value.rotationPower)
            }
        }
    }

    /**
     * Measures whether 2 [ChassisPowers] are roughly equal.
     */
    infix fun epsilonEquals(other: ChassisPowers): Boolean =
        abs(xPower - other.xPower) <= 0.01
            && abs(yPower - other.yPower) <= 0.01
            && abs(rotationPower - other.rotationPower) <= 0.01
}





