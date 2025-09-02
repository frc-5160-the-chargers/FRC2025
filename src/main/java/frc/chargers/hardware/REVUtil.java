package frc.chargers.hardware;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Alert;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

public class REVUtil {
    private REVUtil() {}

    /**
     * Attempts to run the command until no error is produced.
     * If the max attempts are exceeded, an alert will be shown
     * unless if errorMsg is null.
     */
    public static REVLibError retryFor(
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
}
