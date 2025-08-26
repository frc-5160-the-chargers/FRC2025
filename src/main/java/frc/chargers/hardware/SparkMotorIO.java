package frc.chargers.hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

import static com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static edu.wpi.first.math.util.Units.rotationsToRadians;

public class SparkMotorIO<D extends MotorData> implements MotorIO<D> {
    private final SparkBase leader;
    private final DoubleSupplier position;
    private final DoubleSupplier velocity;
    private boolean isAbsoluteEncoder = false;
    private final List<SparkBase> allMotors = new ArrayList<>();

    public SparkMotorIO(SparkBase leader, AbsoluteEncoder encoder) {
        this.leader = leader;
        this.position = encoder::getPosition;
        this.velocity = encoder::getVelocity;
        this.isAbsoluteEncoder = true;
        allMotors.add(leader);
    }

    public SparkMotorIO(SparkBase leader, RelativeEncoder encoder) {
        this.leader = leader;
        this.position = encoder::getPosition;
        this.velocity = encoder::getVelocity;
        allMotors.add(leader);
    }

    @Override
    public void refreshData(D data) {
        data.setNumMotors(allMotors.size());
        data.positionRad = rotationsToRadians(position.getAsDouble());
        if (isAbsoluteEncoder) {
            data.velocityRadPerSec = rotationsToRadians(velocity.getAsDouble());
        } else {
            data.velocityRadPerSec = rotationsPerMinuteToRadiansPerSecond(velocity.getAsDouble());
        }
        var revError = REVLibError.kOk;
        for (int i = 0; i < allMotors.size(); i++) {
            var motor = allMotors.get(i);
            data.appliedVoltages[i] = motor.getAppliedOutput() * motor.getBusVoltage();
            data.supplyCurrents[i] = motor.getOutputCurrent();
            data.motorTemps[i] = motor.getMotorTemperature();
            if (revError == REVLibError.kOk) {
                revError = motor.getLastError();
            }
        }
        if (revError == REVLibError.kOk) {
            data.error = "[None]";
        } else {
            data.error = "[REV] " + revError;
        }
    }

    @Override
    public void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void setTorqueCurrent(double amps) {}

    @Override
    public void setCoastMode(boolean enabled) {
        var config = leader instanceof SparkMax ? new SparkMaxConfig() : new SparkFlexConfig();
        config.idleMode(enabled ? IdleMode.kCoast : IdleMode.kBrake);
        for (var motor : allMotors) {
            motor.configure(config, kNoResetSafeParameters, kNoPersistParameters);
        }
    }
}
