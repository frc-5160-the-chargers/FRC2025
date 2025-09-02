package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.chargers.hardware.MotorInputsAutoLogged;

import java.util.ArrayList;
import java.util.List;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.chargers.hardware.REVUtil.retryFor;

public class SparkIntakeHardware extends IntakeHardware {
    private final SparkBase leader;
    private final int currentLimit;
    private final RelativeEncoder encoder;
    private final SparkBaseConfig config;
    private final List<SparkBase> followers = new ArrayList<>();

    private SparkBaseConfig newConfig(boolean isSparkFlex) {
        return isSparkFlex ? new SparkFlexConfig() : new SparkMaxConfig();
    }

    public SparkIntakeHardware(
        int motorId, boolean isSparkFlex,
        int currentLimit, boolean invert, double reduction
    ) {
        this.leader = isSparkFlex ? new SparkFlex(motorId, kBrushless) : new SparkMax(motorId, kBrushless);
        this.encoder = leader.getEncoder();
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
        config
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        retryFor(
            4,
            "Spark with ID " + motorId + " didn't configure",
            () -> leader.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );
        encoder.setPosition(0);
    }

    public SparkIntakeHardware withFollower(int id, boolean isSparkFlex, boolean invert) {
        var motor = isSparkFlex ? new SparkFlex(id, kBrushless) : new SparkMax(id, kBrushless);
        followers.add(motor);
        retryFor(
            4, "Spark with id " + id + " didn't configure",
            () -> motor.configure(
                newConfig(isSparkFlex)
                        .follow(leader.getDeviceId(), invert)
                        .smartCurrentLimit(currentLimit, 50)
                        .voltageCompensation(12),
                kResetSafeParameters, kPersistParameters
            )
        );
        return this;
    }

    @Override
    public void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        data.refresh(leader, encoder, followers.toArray(new SparkBase[0]));
    }
}
