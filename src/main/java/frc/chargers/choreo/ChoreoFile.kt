package frc.chargers.choreo

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.Serializable
import kotlinx.serialization.Transient
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import java.io.File

@OptIn(ExperimentalSerializationApi::class)
private val JSON_SERIALIZER = Json {
    prettyPrint = true
    prettyPrintIndent = " "
}

/**
 * A utility class that allows adding variables/constants from code
 * into the choreo GUI.
 */
@Serializable
@Suppress("unused")
data class ChoreoFile(
    val name: String,
    val version: Int,
    val type: String,
    private val variables: Variables,
    private val config: JsonObject,
    private val generationFeatures: JsonArray,
) {
    companion object {
        @JvmStatic
        fun from(file: File) =
            JSON_SERIALIZER.decodeFromString<ChoreoFile>(file.readText()).also {
                println("Serialization success!")
                it.file = file
            }
    }

    @Transient
    private var file: File? = null

    fun write() {
        variables.expressions = variables.expressions.toSortedMap()
        variables.poses = variables.poses.toSortedMap()
        file?.writeText(JSON_SERIALIZER.encodeToString(this))
        println("Choreo variable editing finished successfully!")
    }

    fun setVariable(variableName: String, value: Pose2d) {
        variables.poses[variableName] = SerializablePose(
            SerializableValue.withSymbol(value.x, "m"),
            SerializableValue.withSymbol(value.y, "m"),
            SerializableValue.withSymbol(value.rotation.degrees, "deg"),
        )
    }

    fun setVariable(variableName: String, value: Double) {
        variables.expressions[variableName] = SerializableVariable(
            "Number",
            SerializableValue.withoutSymbol(value),
        )
    }

    fun setVariable(variableName: String, value: Distance) {
        variables.expressions[variableName] = SerializableVariable(
            "Distance",
            SerializableValue.withSymbol(value.`in`(Meters), "m")
        )
    }

    fun setVariable(variableName: String, value: Angle) {
        variables.expressions[variableName] = SerializableVariable(
            "Angle",
            SerializableValue.withSymbol(value.`in`(Degrees), "deg")
        )
    }

    fun setVariable(variableName: String, value: LinearVelocity) {
        variables.expressions[variableName] = SerializableVariable(
            "LinVel",
            SerializableValue.withSymbol(value.`in`(MetersPerSecond), "m / s")
        )
    }

    fun setVariable(variableName: String, value: LinearAcceleration) {
        variables.expressions[variableName] = SerializableVariable(
            "LinAcc",
            SerializableValue.withSymbol(value.`in`(MetersPerSecondPerSecond), "m / s / s")
        )
    }

    fun removeVariable(variableName: String) {
        variables.expressions.remove(variableName)
        variables.poses.remove(variableName)
    }
}

@Serializable
data class Variables(
    var expressions: MutableMap<String, SerializableVariable>,
    var poses: MutableMap<String, SerializablePose>
)

@Serializable
data class SerializableValue(val exp: String, val `val`: Double) {
    companion object {
        fun withSymbol(num: Double, symbol: String) =
            SerializableValue("$num $symbol", num)

        fun withoutSymbol(num: Double) =
            SerializableValue(num.toString(), num)
    }
}

@Serializable
data class SerializableVariable(val dimension: String, val `var`: SerializableValue)

@Serializable
data class SerializablePose(val x: SerializableValue, val y: SerializableValue, val heading: SerializableValue)