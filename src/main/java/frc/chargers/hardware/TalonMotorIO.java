package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.traits.CommonTalonWithFOC;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

public class TalonMotorIO<D extends MotorData> implements MotorIO<D> {
    private static final List<BaseStatusSignal> batchRefreshSignals = new ArrayList<>();

    private final CommonTalonWithFOC leader;
    private final List<CommonTalonWithFOC> allMotors = new ArrayList<>();

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final TorqueCurrentFOC torqueOut = new TorqueCurrentFOC(0);

    public TalonMotorIO(CommonTalonWithFOC leader, CommonTalonWithFOC... followers) {
        this.leader = leader;
        allMotors.add(leader);
        PhoenixBatchRefresher.registerRio(
            leader.getPosition(false),
            leader.getVelocity(false)
        );
        for (var motor: followers) {
            this.allMotors.add(motor);
            PhoenixBatchRefresher.registerRio(
                motor.getSupplyVoltage(false),
                motor.getSupplyCurrent(false),
                motor.getDeviceTemp(false),
                motor.getTorqueCurrent(false)
            );
        }
    }

    @Override
    public void refreshData(D data) {
        data.setNumMotors(allMotors.size());
        data.positionRad = rotationsToRadians(leader.getPosition(false).getValueAsDouble());
        data.velocityRadPerSec = rotationsToRadians(leader.getVelocity(false).getValueAsDouble());
        for (int i = 0; i < allMotors.size(); i++) {
            var motor = allMotors.get(i);
            data.appliedVoltages[i] = motor.getSupplyVoltage(false).getValueAsDouble();
            data.supplyCurrents[i] = motor.getSupplyCurrent(false).getValueAsDouble();
            data.motorTemps[i] = motor.getDeviceTemp(false).getValueAsDouble();
            data.torqueCurrents[i] = motor.getTorqueCurrent(false).getValueAsDouble();
        }
    }

    @Override
    public void setVolts(double volts) {
        leader.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setTorqueCurrent(double amps) {
        leader.setControl(torqueOut.withOutput(amps));
    }

    @Override
    public void setCoastMode(boolean enabled) {

    }
}
