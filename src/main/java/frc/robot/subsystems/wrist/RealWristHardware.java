package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.chargers.hardware.MotorDataAutoLogged;
import frc.chargers.hardware.SparkSignals;
import frc.chargers.misc.Retry;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.SparkBase.ControlType.kPosition;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.wrist.WristConsts.MOTOR_ID;
import static frc.robot.subsystems.wrist.WristConsts.ZERO_OFFSET;

public class RealWristHardware extends WristHardware {
    private final SparkMax motor = new SparkMax(MOTOR_ID, kBrushless);
    private final SparkSignals signals = new SparkSignals(motor.getAbsoluteEncoder(), motor);
    private final SparkClosedLoopController pid = motor.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    public RealWristHardware() {
        config
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12);
        config.absoluteEncoder
            .zeroCentered(true)
            .setSparkMaxDataPortConfig();
        Retry.revConfig(4, motor, config);
    }

    @Override
    public void refreshData(MotorDataAutoLogged data) {
        signals.refresh(data);
        data.positionRad -= ZERO_OFFSET.in(Radians);
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        radians += ZERO_OFFSET.in(Radians);
        pid.setReference(radians / (2 * Math.PI), kPosition, kSlot0, feedforwardV);
    }

    @Override
    public void setVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPDGains(double p, double d) {
        config.closedLoop.pid(p, 0, d);
        Retry.revConfig(4, motor, config);
    }
}
