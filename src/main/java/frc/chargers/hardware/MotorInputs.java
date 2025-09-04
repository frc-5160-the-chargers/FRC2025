package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Meters;

/**
 * Represents input data read from a group of motors. <br />
 * This class should periodically be updated with {@code MotorData.refresh()}. <br />
 * Note that a gear ratio should be configured with the motor so that the data here
 * is accurate.
 * <h3>IMPORTANT:</h3>
 * Do not use the raw MotorInputs class - instead, use MotorInputsAutoLogged,
 * which extends this class and adds logging support via Logger.processInputs(). <br />
 * The only exception is if you are extending this class(in that case, do not use MotorInputsAutoLogged).
 */
@AutoLog
public class MotorInputs {
    public double positionRad = 0;
    public double velocityRadPerSec = 0;
    public double[] tempCelsius = new double[0];
    public double[] supplyCurrent = new double[0];
    public double[] appliedVoltage = new double[0];
    public double[] torqueCurrent = new double[0];
    protected String error = "[None]"; // Is a string for logging purposes

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

    private double getVal(BaseStatusSignal signal) {
        var status = signal.getStatus();
        if (!error.equals("[None]") && status != StatusCode.OK) {
            error = "[CTRE] " + error;
        }
        return signal.getValueAsDouble();
    }

    /** Refreshes this motor data with TalonFX or TalonFXS motor(s). */
    public void refresh(boolean isCanivore, CommonTalon leader, CommonTalon... followers) {
        CTREUtil.addLeaderMotorSignals(isCanivore, leader);
        CTREUtil.addFollowerMotorSignals(isCanivore, followers);
        setNumMotors(followers.length + 1);
        positionRad = rotationsToRadians(getVal(leader.getPosition(false)));
        velocityRadPerSec = rotationsToRadians(getVal(leader.getVelocity(false)));
        for (int i = 0; i < followers.length + 1; i++) {
            var motor = i == 0 ? followers[0] : followers[i - 1];
            appliedVoltage[i] = getVal(motor.getSupplyVoltage(false));
            supplyCurrent[i] = getVal(motor.getSupplyCurrent(false));
            tempCelsius[i] = getVal(motor.getDeviceTemp(false));
            torqueCurrent[i] = getVal(motor.getTorqueCurrent(false));
        }
    }

    /** Refreshes this motor data with Spark Max or Spark Flex motor(s). */
    public void refresh(SparkBase leader, RelativeEncoder encoder, SparkBase... followers) {
        positionRad = rotationsToRadians(encoder.getPosition());
        velocityRadPerSec = rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        refreshWithREVMotors(leader, followers);
    }

    /** Refreshes this motor data with Spark Max or Spark Flex motor(s). */
    public void refresh(SparkBase leader, AbsoluteEncoder encoder, SparkBase... followers) {
        positionRad = rotationsToRadians(encoder.getPosition());
        velocityRadPerSec = rotationsToRadians(encoder.getVelocity());
        refreshWithREVMotors(leader, followers);
    }

    public void refresh(DCMotorSim sim) {
        setNumMotors(1);
        positionRad = sim.getAngularPositionRad();
        velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        supplyCurrent[0] = sim.getCurrentDrawAmps();
        appliedVoltage[0] = sim.getInput(0);
    }

    public void refresh(SingleJointedArmSim sim) {
        setNumMotors(1);
        positionRad = sim.getAngleRads();
        velocityRadPerSec = sim.getVelocityRadPerSec();
        supplyCurrent[0] = sim.getCurrentDrawAmps();
        appliedVoltage[0] = sim.getInput(0);
    }

    public void refresh(ElevatorSim sim, Distance drumRadius) {
        setNumMotors(1);
        positionRad = sim.getPositionMeters() / drumRadius.in(Meters);
        velocityRadPerSec = sim.getVelocityMetersPerSecond() / drumRadius.in(Meters);
        supplyCurrent[0] = sim.getCurrentDrawAmps();
        appliedVoltage[0] = sim.getInput(0);
    }

    private void setNumMotors(int length) {
        if (tempCelsius.length == length) return;
        tempCelsius = new double[length];
        supplyCurrent = new double[length];
        appliedVoltage = new double[length];
        torqueCurrent = new double[length];
    }

    private void refreshWithREVMotors(SparkBase leader, SparkBase... followers) {
        setNumMotors(followers.length + 1);
        var currErr = REVLibError.kOk;
        for (int i = 0; i < followers.length + 1; i++) {
            var motor = i == 0 ? leader : followers[i-1];
            appliedVoltage[i] = motor.getAppliedOutput() * motor.getBusVoltage();
            supplyCurrent[i] = motor.getOutputCurrent();
            tempCelsius[i] = motor.getMotorTemperature();
            if (currErr == REVLibError.kOk) {
                currErr = motor.getLastError();
            }
        }
        if (currErr == REVLibError.kOk) {
            error = "[None]";
        } else {
            error = "[REV] " + currErr;
        }
    }
}
