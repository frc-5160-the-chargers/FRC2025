package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.chargers.hardware.MotorInputsAutoLogged;
import frc.chargers.hardware.SimUtil;

public class SimIntakeHardware extends IntakeHardware {
    private final DCMotorSim sim;
    private double inputTorqueCurrent = 0;

    public SimIntakeHardware(DCMotor motorKind, double moi, double reduction) {
        this.sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorKind, moi, reduction),
            motorKind
        );
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
