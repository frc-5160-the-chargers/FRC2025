package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.wpilibj.Alert;
import frc.chargers.misc.Convert;
import frc.chargers.misc.Retry;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/**
 * A utility class that reduces boilerplate around refreshing {@link MotorDataAutoLogged}
 * for a group of TalonFX/TalonFXS motors moving the same mechanism.
 */
@SuppressWarnings("StringConcatenationInLoop")
public class TalonSignals {
    private static final Alert NO_REFRESH_ALERT = new Alert("You might not be calling SignalBatchRefresher.refreshAll().", kError);

    public final BaseStatusSignal position, velocity, voltage;

    private final List<BaseStatusSignal>
        all = new ArrayList<>(),
        motorTemp = new ArrayList<>(),
        supplyCurrent = new ArrayList<>(),
        torqueCurrent = new ArrayList<>();

    /**
     * Creates a new TalonSignals object. To use this class,
     * <b>YOU MUST CALL SignalBatchRefresher.refreshAll() in robotPeriodic().</b>
     */
    public TalonSignals(CommonTalon leader, CommonTalon... followers) {
        boolean isCanivore = leader.getNetwork().isNetworkFD();
        position = leader.getPosition();
        velocity = leader.getVelocity();
        voltage = leader.getMotorVoltage();
        SignalBatchRefresher.register(isCanivore, position, velocity, voltage);
        all.addAll(List.of(position, velocity, voltage));

        addExtSignals(isCanivore, leader);
        for (var follower: followers) {
            addExtSignals(isCanivore, follower);
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
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.positionRad = position.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        inputs.velocityRadPerSec = velocity.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        for (int i = 0; i < numMotors; i++) {
            inputs.supplyCurrent[i] = supplyCurrent.get(i).getValueAsDouble();
            inputs.tempCelsius[i] = motorTemp.get(i).getValueAsDouble();
            inputs.torqueCurrent[i] = torqueCurrent.get(i).getValueAsDouble();
        }
        for (var signal: all) {
            if (signal.getStatus().isOK()) continue;
            inputs.errorAsString += (signal.getStatus() + ",");
        }
        NO_REFRESH_ALERT.set(inputs.tempCelsius[0] == 0.0);
    }

    /**
     * Sets the update frequency of all signals.
     * @param hz The target frequency
     */
    public void setUpdateFrequency(double hz) {
        Retry.ctreConfig(
            4, "Status signal frequency set failed",
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                hz, all.toArray(new BaseStatusSignal[0])
            )
        );
    }

    private void addExtSignals(boolean isCanivore, CommonTalon motor) {
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
