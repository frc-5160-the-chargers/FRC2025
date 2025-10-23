package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.chargers.hardware.MotorDataAutoLogged;
import frc.chargers.hardware.SparkSignals;
import frc.chargers.misc.Retry;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

public class SparkIntakeHardware extends IntakeHardware {
    private final SparkBase leader;
    private final int currentLimit;
    private final SparkSignals signals;
    private final SparkBaseConfig config;

    private SparkBaseConfig newConfig(boolean isSparkFlex) {
        return isSparkFlex ? new SparkFlexConfig() : new SparkMaxConfig();
    }

    public SparkIntakeHardware(
        int motorId, boolean isSparkFlex,
        int currentLimit, boolean invert, double reduction
    ) {
        this.leader = isSparkFlex ? new SparkFlex(motorId, kBrushless) : new SparkMax(motorId, kBrushless);
        this.signals = new SparkSignals(leader.getEncoder(), leader);
        this.currentLimit = currentLimit;
        this.config = newConfig(isSparkFlex);
        config.encoder
            .positionConversionFactor(1 / reduction)
            .velocityConversionFactor(1 / reduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        config
            .inverted(invert)
            .smartCurrentLimit(currentLimit, 50)
            .voltageCompensation(12.0);
        Retry.revConfig(4, leader, config);
        leader.getEncoder().setPosition(0);
    }

    public SparkIntakeHardware withFollower(int id, boolean isSparkFlex, boolean invert) {
        var motor = isSparkFlex ? new SparkFlex(id, kBrushless) : new SparkMax(id, kBrushless);
        var config = newConfig(isSparkFlex)
            .follow(leader.getDeviceId(), invert)
            .smartCurrentLimit(currentLimit, 50)
            .voltageCompensation(12);
        signals.addMotor(motor);
        Retry.revConfig(4, motor, config);
        return this;
    }

    @Override
    public void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void refreshData(MotorDataAutoLogged data) {
        signals.refresh(data);
    }
}
