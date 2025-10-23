package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.chargers.hardware.MotorDataAutoLogged;
import frc.chargers.misc.SimUtil;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConsts.*;

public class SimElevatorHardware extends ElevatorHardware {
    private final ElevatorSim sim = new ElevatorSim(
        MOTOR_KIND, REDUCTION, CARRIAGE_MASS.in(Kilograms),
        RADIUS.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters),
        true,
        0
    );
    private final PIDController pidController = new PIDController(0, 0, 0);
    private double inputV = 0;

    @Override
    public void setPDGains(double p, double d) {
        pidController.setPID(p, 0, d);
    }

    @Override
    public void refreshData(MotorDataAutoLogged data) {
        sim.update(0.02);
        data.positionRad = sim.getPositionMeters() / RADIUS.in(Meters);
        data.velocityRadPerSec = sim.getVelocityMetersPerSecond() / RADIUS.in(Meters);
        data.supplyCurrent[0] = sim.getCurrentDrawAmps();
        data.appliedVolts = inputV;
    }

    @Override
    public void setRadians(double radians, double feedforwardV) {
        setVolts(
            pidController.calculate(
                sim.getPositionMeters() / RADIUS.in(Meters),
                radians
            ) + feedforwardV
        );
    }

    @Override
    public void setVolts(double volts) {
        inputV = SimUtil.currentLimitVoltage(
            volts, MOTOR_KIND, Amps.of(CURRENT_LIMIT),
            RadiansPerSecond.of(
                sim.getVelocityMetersPerSecond() / RADIUS.in(Meters)
            ),
            REDUCTION
        );
        sim.setInputVoltage(inputV);
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}
