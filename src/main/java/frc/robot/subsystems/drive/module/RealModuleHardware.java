// Copyright 2021-2025 FRC 6328
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.chargers.hardware.TalonSignals;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Retry;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.OdoThread;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

import static frc.robot.subsystems.drive.SwerveConsts.*;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX steer motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class RealModuleHardware extends ModuleHardware {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    // Hardware objects
    protected final TalonFX driveTalon, steerTalon;
    protected final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageReq = new VoltageOut(0);
    private final PositionVoltage positionVoltageReq = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageReq = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Odometry thread queues
    private final Queue<Double> timestampQueue, drivePositionQueue, steerPositionQueue;

    // Signals
    private final TalonSignals driveSignals, steerSignals;
    private final StatusSignal<Angle> steerAbsolutePosition;

    public RealModuleHardware(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        boolean isCanivore = TunerConstants.kCANBus.isNetworkFD();
        Logger.recordOutput("Canivore", isCanivore);
        String bus = TunerConstants.DrivetrainConstants.CANBusName;
        this.constants = constants;
        driveTalon = new TalonFX(constants.DriveMotorId, bus);
        steerTalon = new TalonFX(constants.SteerMotorId, bus);
        cancoder = new CANcoder(constants.EncoderId, bus);
        driveSignals = new TalonSignals(isCanivore, driveTalon);
        steerSignals = new TalonSignals(isCanivore, steerTalon);

        configureDevices();

        // We use queues for multithreaded odometry, since advantagekit forces a 0.02 sec cycle time for data updates
        timestampQueue = OdoThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdoThread.getInstance().register(driveSignals.position);
        steerPositionQueue = OdoThread.getInstance().register(steerSignals.position);
        steerAbsolutePosition = cancoder.getAbsolutePosition();
        SignalBatchRefresher.register(isCanivore, steerAbsolutePosition);

        // Configure periodic frames
        driveSignals.setUpdateFreqForAll(50.0);
        steerSignals.setUpdateFreqForAll(50.0);
        BaseStatusSignal.setUpdateFrequencyForAll(
            ODO_FREQUENCY_HZ, driveSignals.position, steerSignals.position
        );
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    }

    @Override
    public void refreshData(ModuleDataAutoLogged inputs) {
        driveSignals.refresh(inputs.drive);
        steerSignals.refresh(inputs.steer);

        inputs.steerAbsolutePos = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
        inputs.odoTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odoDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.odoSteerPositions = steerPositionQueue.stream()
                .map(Rotation2d::fromRotations)
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> voltageReq.withOutput(output);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> voltageReq.withOutput(output);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageReq.withVelocity(velocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
                });
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> positionVoltageReq.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.getRotations());
                });
    }

    @Override
    public void addInstruments(Orchestra orchestra) {
        orchestra.addInstrument(driveTalon);
        orchestra.addInstrument(steerTalon);
        orchestra.addInstrument(cancoder);
    }

    private void configureDevices() {
        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        if (driveConfig.Slot0.kV == 0) {
            // Use a calculated optimistic default if no KV set
            double driveRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
            driveConfig.Slot0.kV = 1 / (DRIVE_MOTOR_TYPE.KvRadPerSecPerVolt / (2 * Math.PI) / driveRatio);
        }
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        Retry.ctreConfig(5, driveTalon, driveConfig);
        Retry.ctreConfig(5, "Drive position was not set", () -> driveTalon.setPosition(0.0, 0.25));

        // Configure steer motor
        var steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Slot0 = constants.SteerMotorGains;
        steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        steerConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            default -> throw new RuntimeException(
                "You are using an unsupported swerve configuration, which this template does not support without manual customization. \n"
                    + "The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");};
        steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.MotorOutput.Inverted = constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        Retry.ctreConfig(5, steerTalon, steerConfig);

        // Configure CANCoder
        var cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
        Retry.ctreConfig(
            5, "Cancoder didn't configure", 
            () -> cancoder.getConfigurator().apply(cancoderConfig)
        );
    }
}
