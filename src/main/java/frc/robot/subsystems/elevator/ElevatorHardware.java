package frc.robot.subsystems.elevator;

import frc.chargers.hardware.MotorDataAutoLogged;

/**
 * A class that controls the hardware powering the elevator(motors and encoders).
 * @see RealElevatorHardware
 * @see SimElevatorHardware
 */
public class ElevatorHardware {
    private double posRequest = 0;

    public void refreshData(MotorDataAutoLogged data) {
        data.positionRad = posRequest;
    }
    
    public void setRadians(double radians, double feedforwardV) {
        posRequest = radians;
    }
    
    public void setVolts(double volts) {}
    
    public void setPDGains(double p, double d) {}
    
    public void zeroEncoder() {}
}
