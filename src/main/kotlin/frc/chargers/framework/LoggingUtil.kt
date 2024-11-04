package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.wpilibj.Timer
import monologue.Logged
import java.nio.ByteBuffer

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
    NullableDoubleHolder.innerValue = value
    log(key, /*struct*/ NullableDoubleHolder, /*value*/ NullableDoubleHolder)
}
/** Logs a nullable Int. */
fun Logged.logNullable(key: String, value: Int?) = logNullable(key, value?.toDouble())
/** Logs a nullable String. */
fun Logged.logNullable(key: String, value: String?) = log(key, value ?: "NULL")

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

private object NullableDoubleHolder: Struct<NullableDoubleHolder> {
    var innerValue: Double? = null

    override fun getTypeClass() = this.javaClass
    override fun getTypeName() = "struct:NullableDouble"
    override fun getSize(): Int = Struct.kSizeDouble + Struct.kSizeBool
    override fun getSchema() = "double value;boolean isNull"
    override fun unpack(bb: ByteBuffer) = this // no struct unpacking supported
    override fun pack(bb: ByteBuffer, value: NullableDoubleHolder) {
        bb.putDouble(innerValue ?: 0.0)
        bb.put(if (innerValue == null) 0 else 1)
    }
}
