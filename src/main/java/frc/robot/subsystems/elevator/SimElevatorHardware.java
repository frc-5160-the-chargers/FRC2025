package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.ElevatorConsts.*;

public class SimElevatorHardware extends ElevatorHardware {
    private final ElevatorSim sim = new ElevatorSim(
        ELEVATOR_SYSTEM, MOTOR_KIND,
        MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters),
        false, 0
    );
    private final PIDController pidController = new PIDController(KP, 0, KD);
    private double inputV = 0;

    @Override
    public void refreshData(ElevatorDataAutoLogged inputs) {
        sim.update(0.02);
        inputs.radians = sim.getPositionMeters() / RADIUS.in(Meters);
        inputs.radiansPerSec = sim.getVelocityMetersPerSecond() / RADIUS.in(Meters);
        inputs.supplyAmps[0] = sim.getCurrentDrawAmps();
        inputs.volts = inputV;
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
        inputV = applyCurrentLimit(volts);
        sim.setInputVoltage(inputV);
    }

    private double applyCurrentLimit(double volts) {
        double radPerSec = sim.getVelocityMetersPerSecond() * REDUCTION / RADIUS.in(Meters);
        double minVolts = MOTOR_KIND.getVoltage(MOTOR_KIND.getTorque(-CURRENT_LIMIT), radPerSec);
        double maxVolts = MOTOR_KIND.getVoltage(MOTOR_KIND.getTorque(CURRENT_LIMIT), radPerSec);
        return MathUtil.clamp(volts, minVolts, maxVolts);
    }

    @Override
    public void zeroEncoder() {
        sim.setState(0, 0);
    }
}