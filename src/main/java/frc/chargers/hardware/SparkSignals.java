package frc.chargers.hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static edu.wpi.first.math.util.Units.rotationsToRadians;

/**
 * A utility class that reduces boilerplate around refreshing {@link MotorDataAutoLogged}
 * for REV Spark Max and Spark Flex motors.
 */
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
        var errTxt = new StringBuilder();
        inputs.setNumMotors(motors.size());
        inputs.appliedVolts = motors.get(0).getAppliedOutput() * motors.get(0).getBusVoltage();
        for (int i = 0; i < motors.size(); i++) {
            var motor = motors.get(i);
            inputs.supplyCurrent[i] = motor.getOutputCurrent();
            inputs.tempCelsius[i] = motor.getMotorTemperature();
            var err = motor.getLastError();
            if (err != REVLibError.kOk) {
                errTxt.append(err).append(",");
            }
        }
        inputs.errorAsString = errTxt.toString();
        if (encoder instanceof RelativeEncoder e) {
            inputs.positionRad = rotationsToRadians(e.getPosition());
            inputs.velocityRadPerSec = rotationsPerMinuteToRadiansPerSecond(e.getVelocity());
        } else if (encoder instanceof AbsoluteEncoder e) {
            inputs.positionRad = rotationsToRadians(e.getPosition());
            inputs.velocityRadPerSec = rotationsToRadians(e.getVelocity());
        }
    }
}
