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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.hardware.TalonSignals;
import frc.chargers.misc.Retry;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.OdoThread;

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
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

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
    private final BaseStatusSignal steerAbsPos;

    public RealModuleHardware(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        boolean isCanivore = TunerConstants.kCANBus.isNetworkFD();
        String bus = TunerConstants.DrivetrainConstants.CANBusName;
        this.constants = constants;
        driveTalon = new TalonFX(constants.DriveMotorId, bus);
        steerTalon = new TalonFX(constants.SteerMotorId, bus);
        cancoder = new CANcoder(constants.EncoderId, bus);
        // Custom utility classes for data updating
        driveSignals = new TalonSignals(isCanivore, driveTalon);
        steerSignals = new TalonSignals(isCanivore, steerTalon);

        configureDevices();

        // We use queues for multithreaded odometry, since advantagekit forces a 0.02 sec cycle time for data updates
        drivePositionQueue = OdoThread.getInstance().register(driveSignals.position);
        steerPositionQueue = OdoThread.getInstance().register(steerSignals.position);
        steerAbsPos = cancoder.getAbsolutePosition();
        SignalBatchRefresher.register(isCanivore, steerAbsPos);

        // Configure periodic frames
        driveSignals.setUpdateFrequency(50.0);
        steerSignals.setUpdateFrequency(50.0);
        Retry.ctreConfig(
            4, "Position signals were not set to 250hz",
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                ODO_FREQUENCY_HZ, driveSignals.position, steerSignals.position
            )
        );
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    }

    @Override
    public void refreshData(ModuleDataAutoLogged inputs) {
        driveSignals.refresh(inputs.drive);
        steerSignals.refresh(inputs.steer);
        inputs.steerAbsolutePos = Rotation2d.fromRotations(steerAbsPos.getValueAsDouble());
        inputs.cachedDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.cachedSteerPositions = steerPositionQueue.stream()
                .map(Rotation2d::fromRotations)
                .toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double voltsOrAmps) {
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> voltageReq.withOutput(voltsOrAmps);
                    case TorqueCurrentFOC -> torqueCurrentReq.withOutput(voltsOrAmps);
                });
    }

    @Override
    public void setSteerOpenLoop(double voltsOrAmps) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> voltageReq.withOutput(voltsOrAmps);
                    case TorqueCurrentFOC -> torqueCurrentReq.withOutput(voltsOrAmps);
                });
    }

    @Override
    public void setDriveVelocity(double radPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(radPerSec);
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageReq.withVelocity(velocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentReq.withVelocity(velocityRotPerSec);
                });
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> positionVoltageReq.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC -> positionTorqueCurrentReq.withPosition(rotation.getRotations());
                });
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
