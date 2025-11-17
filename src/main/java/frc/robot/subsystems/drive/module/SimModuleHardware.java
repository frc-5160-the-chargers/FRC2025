package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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
                .withEncoderOffset(0)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
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
        inputs.cachedDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
        inputs.cachedSteerPositions = simulation.getCachedSteerAbsolutePositions();
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
