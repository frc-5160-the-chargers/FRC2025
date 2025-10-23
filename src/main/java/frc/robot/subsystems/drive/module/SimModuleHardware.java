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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.chargers.misc.SimUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;

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
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // Adjust steer motor PID gains for simulation
                .withSteerMotorGains(new Slot0Configs()
                    .withKP(70)
                    .withKI(0)
                    .withKD(4.5)
                    .withKS(0)
                    .withKV(1.91)
                    .withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(16.0)
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05))
        );

        this.simulation = simulation;
        simulation.useDriveMotorController(new TalonFXSim(super.driveTalon, null));
        simulation.useSteerMotorController(new TalonFXSim(super.steerTalon, super.cancoder));
    }

    @Override
    public void refreshData(ModuleData inputs) {
        super.refreshData(inputs);
        inputs.odoTimestamps = SimUtil.simulateOdoTimestamps();
        inputs.odoDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
        inputs.odoSteerPositions = simulation.getCachedSteerAbsolutePositions();
        Logger.recordOutput("Maple sim pos", simulation.getDriveWheelFinalPosition());
    }

    private record TalonFXSim(TalonFX motor, @Nullable CANcoder encoder) implements SimulatedMotorController {
        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity
        ) {
            motor.getSimState().setSupplyVoltage(12);
            motor.getSimState().setRawRotorPosition(encoderAngle);
            motor.getSimState().setRotorVelocity(encoderVelocity);
            motor.getSimState().setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            if (encoder != null) {
                encoder.getSimState().setRawPosition(mechanismAngle);
                encoder.getSimState().setVelocity(mechanismVelocity);
            }
            return motor.getSimState().getMotorVoltageMeasure();
        }
    }
}
