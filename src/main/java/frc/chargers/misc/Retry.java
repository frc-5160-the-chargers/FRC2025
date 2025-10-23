package frc.chargers.misc;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.Alert;

import java.util.function.Supplier;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

@SuppressWarnings("UnusedReturnValue")
public class Retry {
    /**
     * Attempts to run the command until no error is produced.
     * If the max attempts are exceeded, an alert will be shown
     * unless if errorMsg is null.
     */
    public static StatusCode ctreConfig(
        int maxAttempts,
        String errorMsg,
        Supplier<StatusCode> command
    ) {
        var result = StatusCode.OK;
        for (int i = 0; i < maxAttempts; i++) {
            result = command.get();
            if (result.isOK()) return result;
        }
        if (errorMsg != null) {
            new Alert("[" + result + "] " + errorMsg, kError).set(true);
        }
        return result;
    }

    /**
     * Attempts to run the command until no error is produced.
     * If the max attempts are exceeded, an alert will be shown
     * unless if errorMsg is null.
     */
    public static REVLibError revConfig(
        int maxAttempts,
        String errorMsg,
        Supplier<REVLibError> command
    ) {
        var result = REVLibError.kOk;
        for (int i = 0; i < maxAttempts; i++) {
            result = command.get();
            if (result == REVLibError.kOk) return result;
        }
        if (errorMsg != null) {
            new Alert("[" + result + "] " + errorMsg, kError).set(true);
        }
        return result;
    }

    public static StatusCode ctreConfig(
        int maxAttempts,
        TalonFX motor,
        TalonFXConfiguration config
    ) {
        return ctreConfig(
            maxAttempts,
            motor + " didn't configure",
            () -> motor.getConfigurator().apply(config, 0.025)
        );
    }

    public static REVLibError revConfig(
        int maxAttempts,
        SparkBase motor,
        SparkBaseConfig config
    ) {
        return revConfig(
            maxAttempts,
            motor + " didn't configure",
            () -> motor.configure(config, kResetSafeParameters, kPersistParameters)
        );
    }
}
