package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.hardware.TalonSignals;
import frc.chargers.misc.Convert;
import frc.chargers.misc.Retry;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.OdoThread;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

import static frc.robot.subsystems.drive.SwerveConsts.DRIVE_MOTOR_TYPE;
import static frc.robot.subsystems.drive.SwerveConsts.ODO_FREQUENCY_HZ;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX steer motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class RealModuleHardware extends ModuleHardware {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> consts;

    // Hardware objects
    protected final TalonFX driveTalon, steerTalon;
    protected final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageReq = new VoltageOut(0);
    private final PositionVoltage positionVoltageReq = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageReq = new VelocityVoltage(0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentReq = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentReq = new PositionTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentReq = new VelocityTorqueCurrentFOC(0);

    // Odometry thread queues
    private final Queue<Double> drivePositionQueue, steerPositionQueue;

    // Signals
    private final TalonSignals driveSignals, steerSignals;
    private final BaseStatusSignal steerAbsPos, velocityErr;

    private final double kTAmpsPerNm;

    public RealModuleHardware(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> consts
    ) {
        var bus = TunerConstants.kCANBus;
        this.consts = consts;
        kTAmpsPerNm = 1 / (
            DRIVE_MOTOR_TYPE.KtNMPerAmp / consts.DriveMotorGearRatio
        );
        driveTalon = new TalonFX(consts.DriveMotorId, bus);
        steerTalon = new TalonFX(consts.SteerMotorId, bus);
        cancoder = new CANcoder(consts.EncoderId, bus);
        // Custom utility classes for data updating
        driveSignals = new TalonSignals(driveTalon);
        steerSignals = new TalonSignals(steerTalon);
        // We use queues for multithreaded odometry, since advantagekit forces a 0.02 sec cycle time for data updates
        drivePositionQueue = OdoThread.getInstance().register(driveSignals.position);
        steerPositionQueue = OdoThread.getInstance().register(steerSignals.position);
        steerAbsPos = cancoder.getAbsolutePosition();
        velocityErr = driveTalon.getClosedLoopError();
        SignalBatchRefresher.register(bus.isNetworkFD(), steerAbsPos, velocityErr);

        // Configuration
        configureDevices();
        configureUpdateFrequencies();
    }

    @Override
    public void refreshData(ModuleDataAutoLogged inputs) {
        driveSignals.refresh(inputs.drive);
        steerSignals.refresh(inputs.steer);
        inputs.velocityErrRadPerSec = velocityErr.getValueAsDouble() * Convert.ROTATIONS_TO_RADIANS;
        inputs.steerAbsolutePos = Rotation2d.fromRotations(steerAbsPos.getValueAsDouble());
        inputs.cachedDrivePositionsRad = drivePositionQueue.stream()
            .mapToDouble(it -> it * Convert.ROTATIONS_TO_RADIANS)
            .toArray();
        inputs.cachedSteerPositions = steerPositionQueue.stream()
            .map(Rotation2d::fromRotations)
            .toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double voltsOrAmps) {
        Logger.recordOutput("OutputLol", voltsOrAmps);
        driveTalon.setControl(
            switch (consts.DriveMotorClosedLoopOutput) {
                case Voltage -> voltageReq.withOutput(voltsOrAmps);
                case TorqueCurrentFOC -> torqueCurrentReq.withOutput(voltsOrAmps);
            });
    }

    @Override
    public void setSteerOpenLoop(double voltsOrAmps) {
        steerTalon.setControl(
            switch (consts.SteerMotorClosedLoopOutput) {
                case Voltage -> voltageReq.withOutput(voltsOrAmps);
                case TorqueCurrentFOC -> torqueCurrentReq.withOutput(voltsOrAmps);
            });
    }

    @Override
    public void setDriveVelocity(double radPerSec, double torqueFeedforwardNm) {
        double vel = radPerSec * Convert.RADIANS_TO_ROTATIONS;
        driveTalon.setControl(
            switch (consts.DriveMotorClosedLoopOutput) {
                case Voltage -> velocityVoltageReq.withVelocity(vel);
                case TorqueCurrentFOC -> velocityTorqueCurrentReq
                    .withVelocity(vel)
                    .withFeedForward(torqueFeedforwardNm * kTAmpsPerNm);
            });
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerTalon.setControl(
            switch (consts.SteerMotorClosedLoopOutput) {
                case Voltage -> positionVoltageReq.withPosition(rotation.getRotations());
                case TorqueCurrentFOC -> positionTorqueCurrentReq.withPosition(rotation.getRotations());
            });
    }

    private void configureDevices() {
        // Configure drive motor
        var driveConfig = consts.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        if (consts.DriveMotorGains.kV == 0 && consts.DriveMotorClosedLoopOutput == ClosedLoopOutputType.Voltage) {
            // Use a calculated optimistic default if no KV set
            consts.DriveMotorGains.kV = 1 / (
                DRIVE_MOTOR_TYPE.KvRadPerSecPerVolt
                    * Convert.RADIANS_TO_ROTATIONS
                    / TunerConstants.FrontLeft.DriveMotorGearRatio
            );
        }
        driveConfig.Slot0 = consts.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = consts.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = consts.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -consts.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = consts.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = consts.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        Retry.ctreConfig(5, driveTalon, driveConfig);
        Retry.ctreConfig(5, "Drive position was not set", () -> driveTalon.setPosition(0.0, 0.25));

        // Configure steer motor
        var steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Slot0 = consts.SteerMotorGains;
        steerConfig.Feedback.FeedbackRemoteSensorID = consts.EncoderId;
        steerConfig.Feedback.FeedbackSensorSource = switch (consts.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            default -> throw new RuntimeException(
                "You are using an unsupported swerve configuration, which this template does not support without manual customization. \n"
                    + "The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");};
        steerConfig.Feedback.RotorToSensorRatio = consts.SteerMotorGearRatio;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.MotorOutput.Inverted = consts.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        Retry.ctreConfig(5, steerTalon, steerConfig);

        // Configure CANCoder
        var cancoderConfig = consts.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = consts.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = consts.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
        Retry.ctreConfig(
            5, "Cancoder didn't configure", 
            () -> cancoder.getConfigurator().apply(cancoderConfig)
        );
    }

    private void configureUpdateFrequencies() {
        driveSignals.setUpdateFrequency(50.0);
        steerSignals.setUpdateFrequency(50.0);
        steerAbsPos.setUpdateFrequency(50.0);
        BaseStatusSignal.setUpdateFrequencyForAll(
            ODO_FREQUENCY_HZ, driveSignals.position, steerSignals.position
        );
        Retry.ctreConfig(
            5, "Bus Util wasn't optimized for swerve",
            () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon, cancoder)
        );
    }
}
