package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.chargers.hardware.MotorInputsAutoLogged;
import frc.chargers.hardware.SimUtil;

public class SimIntakeHardware extends IntakeHardware {
    private final DCMotorSim sim;
    private double inputTorqueCurrent = 0;

    public SimIntakeHardware(DCMotorSim sim) {
        this.sim = sim;
    }

    @Override
    public void refreshData(MotorInputsAutoLogged data) {
        sim.update(0.02);
        data.refresh(sim);
        data.torqueCurrent[0] = inputTorqueCurrent;
    }

    @Override
    public void setVolts(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public void setAmps(double amps) {
        inputTorqueCurrent = amps;
        sim.setInputVoltage(
            SimUtil.ampsToVolts(amps, sim.getGearbox(), sim.getAngularVelocity())
        );
    }
}
