@file:Suppress("unused")
package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.utils.waitThenRun
import frc.chargers.wpilibextensions.Cmd
import kotlin.reflect.KProperty

/** A property delegate for a [Double] that can be tuned from advantagescope. */
fun tunable(default: Double, key: String? = null): Tunable<Double> =
    Tunable(key, default, SmartDashboard::getNumber, SmartDashboard::putNumber)

/** A property delegate for a [Boolean] that can be tuned from advantagescope. */
fun tunable(default: Boolean, key: String? = null) =
    Tunable(key, default, SmartDashboard::getBoolean, SmartDashboard::putBoolean)

/** A property delegate for a [Quantity] that can be tuned from advantagescope. */
fun <D: AnyDimension> tunable(default: Quantity<D>, key: String? = null, unit: Quantity<D> = Quantity(1.0)) =
    Tunable(key, default,
        { k, currDefault -> SmartDashboard.getNumber(k, currDefault.inUnit(unit)).ofUnit(unit) },
        { k, v -> SmartDashboard.putNumber(k, v.inUnit(unit)) }
    )

/** A property delegate for [PIDConstants] that can be tuned from advantagescope. */
fun tunable(default: PIDConstants, key: String? = null) =
    Tunable(key, default,
        {k, currDefault ->
            PIDConstants(
                SmartDashboard.getNumber("$k/kP", currDefault.kP),
                SmartDashboard.getNumber("$k/kI", currDefault.kI),
                SmartDashboard.getNumber("$k/kD", currDefault.kD)
            )
        },
        { k, v ->
            SmartDashboard.putNumber("$k/kP", v.kP)
            SmartDashboard.putNumber("$k/kI", v.kI)
            SmartDashboard.putNumber("$k/kD", v.kD)
        }
    )

class Tunable<T>(
    private var key: String?,
    private var current: T,
    private val get: (String, T) -> T,
    private val put: (String, T) -> Unit
) {
    companion object{
        var tuningMode: Boolean = false
    }

    private var onChangeRunnables = mutableListOf<(T) -> Unit>()
    private var path = ""

    /** Runs the following function when the value changes. */
    fun onChange(run: (T) -> Unit): Tunable<T> {
        onChangeRunnables.add(run)
        return this
    }

    /** Detects if the tunable value has changed or not. */
    fun hasChanged(): Boolean =
        path != "" && tuningMode && get(path, current) != current

    // called once on init
    operator fun provideDelegate(thisRef: Any?, property: KProperty<*>): Tunable<T> {
        val propertyName = if (key == null) property.name else key!!
        val group = if (thisRef == null) "OtherTunables" else thisRef::class.simpleName
        this.path = "$group/$propertyName"
        // waits to put the value to the dashboard so that SmartDashboard can initialize
        waitThenRun(1.seconds) {
            put(path, current)
        }
        Trigger(::hasChanged)
            .onTrue(Cmd.runOnce {
                current = get(path, current)
                onChangeRunnables.forEach{ it(current) }
            }.ignoringDisable(true))
        return this
    }
    operator fun getValue(thisRef: Any?, property: KProperty<*>): T = current
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) { current = value }
}