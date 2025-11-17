package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.wpilibj.Alert;
import frc.chargers.misc.Retry;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/**
 * A utility class that reduces boilerplate around refreshing {@link MotorDataAutoLogged}
 * for TalonFX and TalonFXS motors.
 * To use this class, YOU MUST HAVE TalonSignalsRefresher.refreshAll() in robotPeriodic().
 */
public class TalonSignals {
    private static final Alert NO_REFRESH_ALERT = new Alert("You might not be calling SignalBatchRefresher.refreshAll().", kError);

    public final BaseStatusSignal position, velocity, voltage;

    private final List<BaseStatusSignal>
        all = new ArrayList<>(),
        motorTemp = new ArrayList<>(),
        supplyCurrent = new ArrayList<>(),
        torqueCurrent = new ArrayList<>();

    public TalonSignals(boolean isCanivore, CommonTalon leader, CommonTalon... followers) {
        position = leader.getPosition();
        velocity = leader.getVelocity();
        voltage = leader.getMotorVoltage();
        SignalBatchRefresher.register(isCanivore, position, velocity, voltage);
        all.addAll(List.of(position, velocity, voltage));

        addMotor(isCanivore, leader);
        for (var follower: followers) {
            addMotor(isCanivore, follower);
        }
    }

    /**
     * Refreshes a {@link MotorDataAutoLogged} object with data from the signals.
     * @param inputs the motor data to refresh.
     */
    public void refresh(MotorData inputs) {
        int numMotors = motorTemp.size();
        inputs.setNumMotors(numMotors);
        inputs.errorAsString = "";
        inputs.appliedVolts = getValue(voltage, inputs);
        inputs.positionRad = rotationsToRadians(getValue(position, inputs));
        inputs.velocityRadPerSec = rotationsToRadians(getValue(velocity, inputs));
        for (int i = 0; i < numMotors; i++) {
            inputs.supplyCurrent[i] = getValue(supplyCurrent.get(i), inputs);
            inputs.tempCelsius[i] = getValue(motorTemp.get(i), inputs);
            inputs.torqueCurrent[i] = getValue(torqueCurrent.get(i), inputs);
        }
        NO_REFRESH_ALERT.set(inputs.tempCelsius[0] == 0.0);
    }

    /** Sets the update frequency of all signals. */
    public void setUpdateFrequency(double hz) {
        Retry.ctreConfig(
            4, "Status signal frequency set failed",
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                hz, all.toArray(new BaseStatusSignal[0])
            )
        );
    }

    private double getValue(BaseStatusSignal signal, MotorData inputs) {
        var status = signal.getStatus();
        if (status != StatusCode.OK) inputs.errorAsString += (status.toString() + ",");
        return signal.getValueAsDouble();
    }

    private void addMotor(boolean isCanivore, CommonTalon motor) {
        BaseStatusSignal[] signals = {
            motor.getDeviceTemp(), motor.getSupplyCurrent(), motor.getTorqueCurrent()
        };
        motorTemp.add(signals[0]);
        supplyCurrent.add(signals[1]);
        torqueCurrent.add(signals[2]);
        SignalBatchRefresher.register(isCanivore, signals);
        all.addAll(List.of(signals));
    }
}
