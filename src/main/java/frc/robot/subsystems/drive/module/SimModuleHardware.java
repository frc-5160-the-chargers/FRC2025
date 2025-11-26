package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
 * A simulation of module hardware. This class uses MapleSim, a framework
 * that allows for the simulation of collisions and game piece scoring,
 * in addition to the physics of robot movement.
 */
public class SimModuleHardware extends RealModuleHardware {
    // The internal API provided by MapleSim that allows for module simulation.
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
        super.steerTalon.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX44);
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

    /**
     * A utility class that handles simulation of a TalonFX.
     * For swerve, we use "hardware injection sim", where data from physics simulations
     * (in this case, maplesim) is directly injected into the motor, allowing for calls like
     * motor.getPosition() (which wouldn't normally work in sim) to function properly.
     * <br />
     * This kind of simulation works differently for every vendor; so only use this when
     * you're sure that you'll be using TalonFX motors on a mechanism.
     */
    private record TalonFXSim(TalonFX motor, @Nullable CANcoder encoder) implements SimulatedMotorController {
        // This method is repeatedly called by maplesim with the appropriate
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
