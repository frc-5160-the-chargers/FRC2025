package frc.robot.subsystems.elevator;

import frc.chargers.hardware.MotorData;
import org.littletonrobotics.junction.AutoLog;

/**
 * A class that controls the hardware powering the elevator(motors and encoders).
 */
public class ElevatorHardware {
    @AutoLog
    static class ElevatorData extends MotorData {}

    private double posRequest = 0;

    public void refreshData(ElevatorDataAutoLogged inputs) {
        inputs.positionRad = posRequest;
    }

    public void setRadians(double radians, double feedforwardV) {
        posRequest = radians;
    }

    public void setVolts(double volts) {}

    public void setPDGains(double p, double d) {}

    public void zeroEncoder() {}
}