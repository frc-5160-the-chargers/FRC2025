package frc.chargers.hardware;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

public sealed interface MotorError {
    /** Represents no motor error. */
    None NONE = new None();
    record None() implements MotorError {}

    /** Represents an error from a Spark motor. */
    record REV(REVLibError err) implements MotorError {}

    /** Represents an error from a TalonFX motor. */
    record CTRE(StatusCode err) implements MotorError {}
}