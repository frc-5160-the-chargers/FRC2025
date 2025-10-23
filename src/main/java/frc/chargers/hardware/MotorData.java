package frc.chargers.hardware;

import org.littletonrobotics.junction.AutoLog;

/**
 * Represents input data read from a group of motors. <br />
 * You should not instantiate this directly; rather, use the
 * MotorInputsAutoLogged class(which is auto-generated from the @AutoLog annotation)
 * and requires you to build the code first.
 */
@AutoLog
class MotorData {
    public String errorAsString = "";
    public double positionRad = 0;
    public double velocityRadPerSec = 0;
    public double appliedVolts = 0;
    public double[] tempCelsius = new double[1];
    public double[] supplyCurrent = new double[1];
    public double[] torqueCurrent = new double[1];

    public boolean ok() {
        return errorAsString.isEmpty();
    }

    public void setNumMotors(int length) {
        if (tempCelsius.length == length) return;
        tempCelsius = new double[length];
        supplyCurrent = new double[length];
        torqueCurrent = new double[length];
    }
}
