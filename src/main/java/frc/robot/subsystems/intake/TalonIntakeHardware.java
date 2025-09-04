package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.chargers.hardware.MotorInputsAutoLogged;

import java.util.ArrayList;
import java.util.List;

import static frc.chargers.hardware.CTREUtil.retryFor;

public class TalonIntakeHardware extends IntakeHardware {
    private final TalonFX leader;
    private final boolean isCanivore;
    private final TalonFXConfiguration config;
    private final List<TalonFX> followers = new ArrayList<>();
    private final VoltageOut voltageOut = new VoltageOut(0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC torqueOut = new TorqueCurrentFOC(0).withUpdateFreqHz(0);

    public TalonIntakeHardware(
        int motorId, int currentLimit,
        boolean invert, boolean isCanivore, double reduction
    ) {
        this.isCanivore = isCanivore;
        this.leader = new TalonFX(motorId);
        voltageOut.EnableFOC = true;
        config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = reduction;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        retryFor(
            4, leader + " didn't configure",
            () -> leader.getConfigurator().apply(config)
        );
        retryFor(
            4, leader + " didn't optimize bus util",
            leader::optimizeBusUtilization
        );
    }

    public TalonIntakeHardware withFollower(int id, boolean invert) {
        var follower = new TalonFX(id);
        retryFor(
            4, follower + " didn't configure",
            () -> follower.getConfigurator().apply(config)
        );
        follower.setControl(new Follower(leader.getDeviceID(), invert));
        followers.add(follower);
        return this;
    }

    @Override
    public void setVolts(double volts) {
        leader.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setAmps(double amps) {
        leader.setControl(torqueOut.withOutput(amps));
    }

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        data.refresh(isCanivore, leader, followers.toArray(new TalonFX[0]));
    }
}
