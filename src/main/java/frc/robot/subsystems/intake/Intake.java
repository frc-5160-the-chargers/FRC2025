package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    private final IntakeHardware io;
    private final MotorInputsAutoLogged data = new MotorInputsAutoLogged();
    private final String name;

    public Intake(String name, IntakeHardware io) {
        super.setName(name);
        this.name = name;
        this.io = io;
    }

    public Command setVoltageCmd(Voltage voltage) {
        return this.run(() -> io.setVolts(voltage.in(Volts)))
                .withName("Set Voltage (" + voltage + ", " + name + ")");
    }

    public Command setTorqueCurrentCmd(Current current) {
        return this.run(() -> io.setAmps(current.in(Amps)))
                .withName("Set Current (" + current + ", " + name + ")");
    }

    public Trigger supplyCurrentExceeds(Current threshold) {
        return new Trigger(() -> {
            for (var current: data.supplyCurrent) {
                if (current > threshold.in(Amps)) {
                    return true;
                }
            }
            return false;
        });
    }

    public AngularVelocity velocity() {
        return RadiansPerSecond.of(data.velocityRadPerSec);
    }

    @Override
    public void periodic() {
        io.refreshData(data);
        // If real/sim mode, log the data.
        // If replay mode, overrides the data with data from the log file instead.
        Logger.processInputs(name, data);
    }
}
