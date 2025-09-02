package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.chargers.hardware.MotorInputsAutoLogged;
import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.chargers.hardware.REVUtil.retryFor;
import static frc.robot.subsystems.elevator.ElevatorConsts.*;

public class RealElevatorHardware extends ElevatorHardware {
    private final SparkBaseConfig leaderConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(CURRENT_LIMIT)
            .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
            .inverted(LEADER_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkBaseConfig followerConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(CURRENT_LIMIT)
            .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(FOLLOWER_INVERTED)
            .follow(LEADER_MOTOR_ID, true);

    private final SparkMax leader = new SparkMax(LEADER_MOTOR_ID, kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkMax follower = new SparkMax(FOLLOWER_MOTOR_ID, kBrushless);

    public RealElevatorHardware() {
        setPDGains(0, 0); // also configures the motor
        retryFor(
            4, "Elevator follower motor didn't configure",
            () -> follower.configure(followerConfig, kResetSafeParameters, kPersistParameters)
        );
    }

    @Override
    public void setPDGains(double p, double d) {
        leaderConfig.closedLoop.pid(p, 0, d, ClosedLoopSlot.kSlot0);
        retryFor(
            4, "Elevator lead motor didn't configure",
            () -> leader.configure(leaderConfig, kResetSafeParameters, kPersistParameters)
        );
    }

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        data.refresh(leader, encoder, follower);
    }

    @Override
    public void setAngularPosition(double radians, double feedforward) {
        leader.getClosedLoopController().setReference(
            radians / 2 * Math.PI,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward
        );
    }

    @Override
    public void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void zeroEncoder() {
        retryFor(4, "Elevator encoder didn't zero", () -> encoder.setPosition(0));
    }
}
