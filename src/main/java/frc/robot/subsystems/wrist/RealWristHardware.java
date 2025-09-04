package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.chargers.hardware.MotorInputsAutoLogged;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.SparkBase.ControlType.kPosition;
import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Radians;
import static frc.chargers.hardware.REVUtil.retryFor;
import static frc.robot.subsystems.wrist.WristConsts.MOTOR_ID;
import static frc.robot.subsystems.wrist.WristConsts.ZERO_OFFSET;

public class RealWristHardware extends WristHardware {
    private final SparkMax motor = new SparkMax(MOTOR_ID, kBrushless);
    private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder();
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
        configMotor();
    }

    private void configMotor() {
        retryFor(
            4, "Wrist motor didn't configure",
            () -> motor.configure(config, kResetSafeParameters, kPersistParameters)
        );
    }

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        data.refresh(motor, encoder);
        data.positionRad -= ZERO_OFFSET.in(Radians);
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        pid.setReference((radians + ZERO_OFFSET.in(Radians)) / (2 * Math.PI), kPosition, kSlot0, feedforwardV);
    }

    @Override
    public void setVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPDGains(double p, double d) {
        config.closedLoop.pid(p, 0, d);
        configMotor();
    }
}
