package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.chargers.hardware.MotorInputsAutoLogged;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static frc.robot.subsystems.wrist.WristConsts.*;

public class SimWristHardware extends WristHardware {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        MOTOR_KIND,
        REDUCTION,
        MOI.in(KilogramSquareMeters),
        0.5,
        -Math.PI,
        Math.PI,
        true,
        0
    );
    private final PIDController pid = new PIDController(0, 0, 0);

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        data.refresh(sim);
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        setVolts(pid.calculate(sim.getAngleRads(), radians) + feedforwardV);
    }

    @Override
    public void setVolts(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public void setPDGains(double p, double d) {
        pid.setPID(p, 0, d);
    }
}
