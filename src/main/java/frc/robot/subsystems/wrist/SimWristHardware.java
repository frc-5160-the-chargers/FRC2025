package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.chargers.hardware.MotorDataAutoLogged;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static frc.robot.subsystems.wrist.WristConsts.*;

public class SimWristHardware extends WristHardware {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        MOTOR_KIND,
        REDUCTION,
        MOI.in(KilogramSquareMeters),
        0.5,
        -Math.PI, Math.PI,
        true,
        0
    );
    private final PIDController pid = new PIDController(0, 0, 0);
    private double outputV = 0;

    @Override
    public void refreshData(MotorDataAutoLogged data) {
        data.positionRad = sim.getAngleRads();
        data.velocityRadPerSec = sim.getVelocityRadPerSec();
        data.supplyCurrent[0] = sim.getCurrentDrawAmps();
        data.appliedVolts = outputV;
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        setVolts(pid.calculate(sim.getAngleRads(), radians) + feedforwardV);
    }

    @Override
    public void setVolts(double volts) {
        outputV = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setPDGains(double p, double d) {
        pid.setPID(p, 0, d);
    }
}
