package frc.robot.subsystems.wrist;

import frc.chargers.hardware.MotorDataAutoLogged;

public class WristHardware {
    private double radians = 0;

    public void refreshData(MotorDataAutoLogged data) {
        data.positionRad = radians;
    }

    public void setRadians(double radians, double feedforwardV) {
        this.radians = radians;
    }

    public void setVolts(double volts) {}

    public void setPDGains(double p, double d) {}
}
