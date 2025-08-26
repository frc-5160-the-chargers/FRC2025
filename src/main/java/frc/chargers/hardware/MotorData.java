package frc.chargers.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.CommonTalonWithFOC;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static edu.wpi.first.math.util.Units.rotationsToRadians;

@AutoLog
public class MotorData {
    protected String error = "[None]";
    public double positionRad = 0;
    public double velocityRadPerSec = 0;
    public double[] motorTemps = new double[0];
    public double[] supplyCurrents = new double[0];
    public double[] appliedVoltages = new double[0];
    public double[] torqueCurrents = new double[0];

    public void refresh(CommonTalonWithFOC leader, CommonTalonWithFOC... followers) {
        setNumMotors(followers.length + 1);
        positionRad = rotationsToRadians(leader.getPosition(false).getValueAsDouble());
        velocityRadPerSec = rotationsToRadians(leader.getVelocity(false).getValueAsDouble());
        for (int i = 0; i < followers.length + 1; i++) {
            var motor = i == 0 ? followers[0] : followers[i - 1];
            appliedVoltages[i] = motor.getSupplyVoltage(false).getValueAsDouble();
            supplyCurrents[i] = motor.getSupplyCurrent(false).getValueAsDouble();
            motorTemps[i] = motor.getDeviceTemp(false).getValueAsDouble();
            torqueCurrents[i] = motor.getTorqueCurrent(false).getValueAsDouble();
        }
    }
    
    public void refresh(SparkBase leader, RelativeEncoder encoder, SparkBase... followers) {
        positionRad = rotationsToRadians(encoder.getPosition());
        velocityRadPerSec = rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        refreshWithREVMotors(leader, followers);
    }
    
    public void refresh(SparkBase leader, AbsoluteEncoder encoder, SparkBase... followers) {
        positionRad = rotationsToRadians(encoder.getPosition());
        velocityRadPerSec = rotationsToRadians(encoder.getVelocity());
        refreshWithREVMotors(leader, followers);
    }

    public MotorError getError() {
        if (error.startsWith("[REV] ")) {
            String errStr = error.substring(6);
            return new MotorError.REV(REVLibError.valueOf(errStr));
        } else if (error.startsWith("[CTRE] ")) {
            String errStr = error.substring(7);
            return new MotorError.CTRE(StatusCode.valueOf(errStr));
        } else {
            return MotorError.NONE;
        }
    }
    
    
    private void refreshWithREVMotors(SparkBase leader, SparkBase... followers) {
        setNumMotors(followers.length + 1);
        var revError = REVLibError.kOk;
        for (int i = 0; i < followers.length + 1; i++) {
            var motor = i == 0 ? leader : followers[i-1];
            appliedVoltages[i] = motor.getAppliedOutput() * motor.getBusVoltage();
            supplyCurrents[i] = motor.getOutputCurrent();
            motorTemps[i] = motor.getMotorTemperature();
            if (revError == REVLibError.kOk) {
                revError = motor.getLastError();
            }
        }
        if (revError == REVLibError.kOk) {
            error = "[None]";
        } else {
            error = "[REV] " + revError;
        }
    }

    private void setNumMotors(int length) {
        if (motorTemps.length == length) return;
        motorTemps = new double[length];
        supplyCurrents = new double[length];
        appliedVoltages = new double[length];
        torqueCurrents = new double[length];
    }
}
