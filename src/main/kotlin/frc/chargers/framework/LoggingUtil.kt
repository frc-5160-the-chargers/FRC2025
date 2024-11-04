package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.Timer
import monologue.Annotations.SingletonLogged
import monologue.Logged
import java.nio.ByteBuffer
import kotlin.internal.LowPriorityInOverloadResolution

/** A utility object that allows you to log from subcomponents on the robot. */
@SingletonLogged(key = "")
object GlobalLog: Logged

/**
 * Logs the latency of a certain function.
 * There is no runtime overhead for logging latency like this.
 */
inline fun <T> Logged.logLatency(key: String, toRun: () -> T): T {
    val startTime = Timer.getFPGATimestamp()
    val returnValue = toRun()
    log("$key(MS)", (Timer.getFPGATimestamp() - startTime) * 1000)
    return returnValue
}

/** Logs a nullable Double. */
fun Logged.logNullable(key: String, value: Double?) {
    NullableNumberHolder.innerValue = value
    log(key, NullableNumberHolder.struct, NullableNumberHolder)
}
/** Logs a nullable Int. */
@LowPriorityInOverloadResolution
fun Logged.logNullable(key: String, value: Int?) = logNullable(key, value?.toDouble())

/** Logs a list of doubles. */
fun Logged.logList(key: String, value: List<Double>) {
    val internalArray = doubleArrays[value.size]
    // longer than 8 items; log through .toDoubleArray()
    if (internalArray == null) {
        log(key, value.toDoubleArray())
    } else {
        for (i in value.indices) internalArray[i] = value[i]
        log(key, internalArray)
    }
}
/** Logs a list of Quantities. */
fun <D: AnyDimension> Logged.logList(key: String, unit: Quantity<D>, value: List<Quantity<D>>) {
    val internalArray = doubleArrays[value.size]
    // longer than 8 items; log through .toDoubleArray()
    if (internalArray == null) {
        log(key, value.map { it.inUnit(unit) }.toDoubleArray())
    } else {
        for (i in value.indices) internalArray[i] = value[i].inUnit(unit)
        log(key, internalArray)
    }
}

// a map between a double array's size and the respective empty array.
val doubleArrays = mutableMapOf<Int, DoubleArray>().apply {
    for (i in 0..8) { put(i, DoubleArray(i)) }
}

private object NullableNumberHolder: StructSerializable {
    var innerValue: Double? = null

    @JvmField val struct = object: Struct<NullableNumberHolder> {
        override fun getTypeClass() = NullableNumberHolder::class.java
        override fun getTypeName() = "NullableNumber"
        override fun getTypeString() = "struct:NullableNumber"
        override fun getSize(): Int = Struct.kSizeDouble + Struct.kSizeBool
        override fun getSchema() = "bool isNull;double value;"
        override fun unpack(bb: ByteBuffer) = this@NullableNumberHolder // unpacking not supported atm
        override fun pack(bb: ByteBuffer, value: NullableNumberHolder) {
            bb.put(if (value.innerValue == null) 1 else 0)
            bb.putDouble(value.innerValue ?: 0.0)
        }
    }
}
