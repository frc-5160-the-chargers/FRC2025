package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.chargers.misc.RobotMode;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class MapleSimSwerveHardware extends SwerveHardware {
    private final Pigeon2SimState gyroSim;
    private final SwerveDriveSimulation mapleSim;

    private MapleSimSwerveHardware(
        SwerveDriveSimulation mapleSim, SwerveDrivetrain<?, ?, ?> drivetrain
    ) {
        super(drivetrain);
        this.mapleSim = mapleSim;
        this.gyroSim = drivetrain.getPigeon2().getSimState();
    }

    /** Creates a variant of SwerveHardware that supports collisions in sim. */
    public static MapleSimSwerveHardware from(
        SwerveDriveSimulation mapleSim,
        SwerveDrivetrainConstants driveConsts,
        SwerveModuleConstants<?, ?, ?>... moduleConsts
    ) {
        if (RobotMode.isSim()) {
            for (var config: moduleConsts) {
                config.EncoderOffset = 0;
                config.DriveMotorInverted = false;
                config.SteerMotorInverted = false;
                config.EncoderInverted = false;
            }
        }
        var impl = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            driveConsts, moduleConsts
        );
        if (RobotMode.isSim()) {
            SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
            for (int i = 0; i < 4; i++) {
                var mod = impl.getModule(i);
                var sim = mapleSim.getModules()[i];
                sim.useDriveMotorController(new TalonFXSim(mod.getDriveMotor(), null));
                sim.useSteerMotorController(new TalonFXSim(mod.getSteerMotor(), mod.getEncoder()));
            }
        }
        return new MapleSimSwerveHardware(mapleSim, impl);
    }

    @Override
    public void refreshData(SwerveDataAutoLogged data) {
        super.refreshData(data);
        if (!RobotMode.isSim()) return;
        // "Injects" data into the gyro, effectively overriding the value of getRotation3d().
        gyroSim.setRawYaw(mapleSim.getSimulatedDriveTrainPose().getRotation().getMeasure());
        var vel = mapleSim.getDriveTrainSimulatedChassisSpeedsRobotRelative();
        gyroSim.setAngularVelocityZ(RadiansPerSecond.of(vel.omegaRadiansPerSecond));
    }

    @Override
    public void resetNotReplayedPose(Pose2d pose) {
        mapleSim.setSimulationWorldPose(pose);
        drivetrain.resetTranslation(pose.getTranslation());
    }

    /**
     * A utility class that handles simulation of a TalonFX.
     * For swerve, we use "hardware injection simulation", where data from physics simulations
     * (in this case, maplesim) is directly injected into the motor, allowing for calls like
     * motor.getPosition() (which wouldn't normally work in sim) to function properly.
     * <br />
     * This kind of simulation works differently for every vendor; so only use this when
     * you're sure that you'll be using TalonFX motors on a mechanism.
     */
    private record TalonFXSim(TalonFX motor, @Nullable CANcoder encoder) implements SimulatedMotorController {
        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle, // position w/ gearing
            AngularVelocity mechanismVelocity,
            Angle encoderAngle, // position without gearing
            AngularVelocity encoderVelocity
        ) {
            if (encoder != null) {
                encoder.getSimState().setRawPosition(mechanismAngle);
                encoder.getSimState().setVelocity(mechanismVelocity);
            }
            motor.getSimState().setRawRotorPosition(encoderAngle);
            motor.getSimState().setRotorVelocity(encoderVelocity);
            return motor.getSimState().getMotorVoltageMeasure();
        }
    }
}
