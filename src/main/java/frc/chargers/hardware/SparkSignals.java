package frc.chargers.hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import frc.chargers.misc.Convert;

import java.util.ArrayList;
import java.util.List;

/**
 * A utility class that reduces boilerplate around refreshing {@link MotorDataAutoLogged}
 * for a group of REV spark max/spark flex motors moving the same mechanism.
 */
@SuppressWarnings("StringConcatenationInLoop")
public class SparkSignals {
    private final List<SparkBase> motors = new ArrayList<>();
    private final Object encoder;

    public SparkSignals(RelativeEncoder encoder, SparkBase leader, SparkBase... followers) {
        motors.add(leader);
        motors.addAll(List.of(followers));
        this.encoder = encoder;
    }

    public SparkSignals(AbsoluteEncoder encoder, SparkBase leader, SparkBase... followers) {
        motors.add(leader);
        motors.addAll(List.of(followers));
        this.encoder = encoder;
    }

    /**
     * Refreshes a {@link MotorDataAutoLogged} object with data from the signals.
     * @param inputs the motor data to refresh.
     */
    public void refresh(MotorData inputs) {
        inputs.setNumMotors(motors.size());
        inputs.errorAsString = "";
        inputs.appliedVolts = motors.get(0).getAppliedOutput() * motors.get(0).getBusVoltage();
        for (int i = 0; i < motors.size(); i++) {
            var motor = motors.get(i);
            inputs.supplyCurrent[i] = motor.getOutputCurrent();
            inputs.tempCelsius[i] = motor.getMotorTemperature();
            var err = motor.getLastError();
            if (err != REVLibError.kOk) {
                inputs.errorAsString += (err + ",");
            }
        }
        if (encoder instanceof RelativeEncoder e) {
            inputs.positionRad = e.getPosition() * Convert.ROTATIONS_TO_RADIANS;
            inputs.velocityRadPerSec = e.getVelocity() * Convert.RPM_TO_RADIANS_PER_SECOND;
        } else if (encoder instanceof AbsoluteEncoder e) {
            inputs.positionRad = e.getPosition() * Convert.ROTATIONS_TO_RADIANS;
            inputs.velocityRadPerSec = e.getVelocity() * Convert.ROTATIONS_TO_RADIANS;
        }
    }
}
