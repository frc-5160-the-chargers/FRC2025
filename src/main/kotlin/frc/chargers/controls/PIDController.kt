package frc.chargers.controls

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController

/**
 * Creates a [PIDController] with [PIDConstants].
 */
fun PIDController(constants: PIDConstants): PIDController =
    PIDController(constants.kP, constants.kI, constants.kD)

/**
 * A getter/setter property for a [PIDController]'s [PIDConstants].
 */
var PIDController.constants: PIDConstants
    get() = PIDConstants(p, i, d)
    set(value){
        setPID(value.kP, value.kI, value.kD)
    }