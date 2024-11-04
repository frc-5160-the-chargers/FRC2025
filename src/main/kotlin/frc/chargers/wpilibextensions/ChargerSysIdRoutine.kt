package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.units.Units as WPIUnits
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.utils.units.VoltageRate
import monologue.Monologue

/** A sysid routine that accepts kmeasure quantities. */
class ChargerSysIdRoutine(
    subsystem: Subsystem,
    setVoltage: (Voltage) -> Unit,
    rampRate: VoltageRate? = null,
    stepVoltage: Voltage? = null,
    timeout: Time? = null,
): SysIdRoutine(
    Config(
        if (rampRate != null) {
            WPIUnits.Volts.per(WPIUnits.Seconds).of(rampRate.inUnit(volts / seconds))
        } else null,
        if (stepVoltage != null) WPIUnits.Volts.of(stepVoltage.inUnit(volts)) else null,
        if (timeout != null) WPIUnits.Seconds.of(timeout.inUnit(seconds)) else null
    ) { routineState -> Monologue.log("${subsystem.name}/sysIdRoutineState", routineState.toString()) },
    Mechanism({ wpiV -> setVoltage(wpiV.`in`(WPIUnits.Volts).ofUnit(volts)) }, null, subsystem)
)