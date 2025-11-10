// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.chargers.misc.SimUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class SimModuleHardware extends RealModuleHardware {
    private final SwerveModuleSimulation simulation;

    public SimModuleHardware(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
        SwerveModuleSimulation simulation
    ) {
        super(
            constants
                // Disable encoder offsets
                .withEncoderOffset(0)
                // Disable inversions
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
        );

        this.simulation = simulation;
        simulation.useDriveMotorController(new TalonFXSim(super.driveTalon, null));
        simulation.useSteerMotorController(new TalonFXSim(super.steerTalon, super.cancoder));
    }

    @Override
    public void refreshData(ModuleDataAutoLogged inputs) {
        super.refreshData(inputs);
        inputs.odoTimestamps = SimUtil.simulateOdoTimestamps();
        inputs.odoDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
        inputs.odoSteerPositions = simulation.getCachedSteerAbsolutePositions();
    }

    private record TalonFXSim(TalonFX motor, @Nullable CANcoder encoder) implements SimulatedMotorController {
        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity
        ) {
            motor.getSimState().setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            motor.getSimState().setRawRotorPosition(encoderAngle);
            motor.getSimState().setRotorVelocity(encoderVelocity);
            if (encoder != null) {
                encoder.getSimState().setRawPosition(mechanismAngle);
                encoder.getSimState().setVelocity(mechanismVelocity);
            }
            return motor.getSimState().getMotorVoltageMeasure();
        }
    }
}
