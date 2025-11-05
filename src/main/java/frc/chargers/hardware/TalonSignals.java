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
 * A utility class that reduces boilerplate around refreshing MotorInputs
 * for CTRE TalonFX and TalonFXS motors.
 * To use this class, YOU MUST HAVE TalonSignalsRefresher.refreshAll() in robotPeriodic().
 */
public class TalonSignals {
    public final boolean isCanivore;
    public final BaseStatusSignal position, velocity, voltage;
    private final List<BaseStatusSignal>
        all = new ArrayList<>(),
        motorTemp = new ArrayList<>(),
        supplyCurrent = new ArrayList<>(),
        torqueCurrent = new ArrayList<>();

    private final Alert noRefreshAlert = new Alert("You might not be calling SignalBatchRefresher.refreshAll().", kError);

    public TalonSignals(boolean isCanivore, CommonTalon leader, CommonTalon... followers) {
        this.isCanivore = isCanivore;
        position = leader.getPosition();
        velocity = leader.getVelocity();
        voltage = leader.getMotorVoltage();
        SignalBatchRefresher.register(isCanivore, position, velocity, voltage);
        all.addAll(List.of(position, velocity, voltage));

        addMotor(leader);
        for (var follower: followers) {
            addMotor(follower);
        }
    }

    /**
     * Refreshes a MotorInputs object with data from the signals
     * @param inputs the MotorInputs to refresh.
     */
    public void refresh(MotorDataAutoLogged inputs) {
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
        noRefreshAlert.set(inputs.tempCelsius[0] == 0.0);
    }

    /** Adds a follower motor to the signals. */
    public void addMotor(CommonTalon motor) {
        BaseStatusSignal[] signals = {
            motor.getDeviceTemp(), motor.getSupplyCurrent(), motor.getTorqueCurrent()
        };
        motorTemp.add(signals[0]);
        supplyCurrent.add(signals[1]);
        torqueCurrent.add(signals[2]);
        SignalBatchRefresher.register(isCanivore, signals);
        all.addAll(List.of(signals));
    }

    /** Sets the update frequency of all signals. */
    public void setUpdateFreqForAll(double hz) {
        Retry.ctreConfig(
            4, "Status signal frequency set failed",
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                hz, all.toArray(new BaseStatusSignal[0])
            )
        );
    }

    private double getValue(BaseStatusSignal signal, MotorDataAutoLogged inputs) {
        var status = signal.getStatus();
        if (status != StatusCode.OK) inputs.errorAsString += (status.toString() + ",");
        return signal.getValueAsDouble();
    }
}
