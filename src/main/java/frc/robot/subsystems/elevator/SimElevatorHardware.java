package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.chargers.hardware.MotorInputsAutoLogged;
import frc.chargers.hardware.SimUtil;

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

    @Override
    public void setPDGains(double p, double d) {
        pidController.setPID(p, 0, d);
    }

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        sim.update(0.02);
        data.refresh(sim, RADIUS);
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
        volts = SimUtil.currentLimitVoltage(
            volts, MOTOR_KIND, Amps.of(CURRENT_LIMIT),
            RadiansPerSecond.of(
                sim.getVelocityMetersPerSecond() / RADIUS.in(Meters)
            ),
            REDUCTION
        );
        sim.setInputVoltage(volts);
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}
