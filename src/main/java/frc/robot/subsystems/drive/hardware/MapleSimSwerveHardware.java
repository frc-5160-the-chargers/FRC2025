package frc.robot.subsystems.drive.hardware;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.SwerveConsts.MODULE_CONSTS_CHOICES;

public class MapleSimSwerveHardware extends SwerveHardware {
    private final Pigeon2SimState gyroSim;
    private final SwerveDriveSimulation mapleSim;

    private MapleSimSwerveHardware(SwerveDriveSimulation mapleSim) {
        this.mapleSim = mapleSim;
        this.gyroSim = super.drivetrain.getPigeon2().getSimState();
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
        for (int i = 0; i < 4; i++) {
            var mod = super.drivetrain.getModule(i);
            var sim = mapleSim.getModules()[i];
            mod.getSteerMotor().getSimState().setMotorType(MotorType.KrakenX44);
            sim.useDriveMotorController(new TalonFXSim(mod.getDriveMotor(), null));
            sim.useSteerMotorController(new TalonFXSim(mod.getSteerMotor(), mod.getEncoder()));
        }
    }

    /** Creates a variant of SwerveHardware that supports collisions in sim. */
    public static MapleSimSwerveHardware create(SwerveDriveSimulation mapleSim) {
        // offsets that aren't applicable to sim must be cleared before the swerve hardware is created.
        for (var config: MODULE_CONSTS_CHOICES) {
            config.EncoderOffset = 0;
            config.DriveMotorInverted = false;
            config.SteerMotorInverted = false;
            config.EncoderInverted = false;
        }
        return new MapleSimSwerveHardware(mapleSim);
    }

    @Override
    public void refreshData(SwerveDataAutoLogged data) {
        super.refreshData(data);
        // The "true" pose of the robot, without odometry drift.
        var truePose = mapleSim.getSimulatedDriveTrainPose();
        var vel = mapleSim.getDriveTrainSimulatedChassisSpeedsRobotRelative();
        Logger.recordOutput("SwerveDrive/TruePose", truePose);
        // "Injects" data into the gyro, overriding the value of getYaw().
        gyroSim.setRawYaw(truePose.getRotation().getMeasure());
        gyroSim.setAngularVelocityZ(RadiansPerSecond.of(vel.omegaRadiansPerSecond));
    }

    @Override
    public void resetNotReplayedPose(Pose2d pose) {
        mapleSim.setSimulationWorldPose(pose);
        super.drivetrain.resetTranslation(pose.getTranslation());
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
