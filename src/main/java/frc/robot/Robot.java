package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.commands.CmdLogger;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.RobotMode;
import frc.chargers.misc.Tracer;
import frc.robot.components.DriverController;
import frc.robot.components.vision.Camera;
import frc.robot.constants.BuildConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.util.List;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

@SuppressWarnings({"FieldCanBeLocal", "DataFlowIssue"})
public class Robot extends LoggedRobot {
    private static final double SIM_UPDATE_PERIOD = 0.002;

    private final SwerveDrive drive = new SwerveDrive("SwerveDrive");
    private final Elevator elevator = new Elevator();
    private final DriverController controller = new DriverController();

    private final List<Camera> cameras = List.of(
//        new Camera(VisionConsts.FR_CONSTS, drive::truePose),
//        new Camera(VisionConsts.FL_CONSTS, drive::truePose)
    );

    public Robot() {
        // A / (m/s)
        initLogging();

        drive.setDefaultCommand(
            drive.driveCmd(controller::getSwerveRequest)
        );
        elevator.setDefaultCommand(elevator.idleCmd());
        DriverStation.silenceJoystickConnectionWarning(true);
        PortForwarder.add(5800, "photonvision.local", 5800);
        if (RobotMode.isSim()) {
            SimulatedArena.overrideSimulationTimings(Seconds.of(SIM_UPDATE_PERIOD), 1);
            new Notifier(SimulatedArena.getInstance()::simulationPeriodic)
                .startPeriodic(SIM_UPDATE_PERIOD); // Updates MapleSim periodically.
        }
        drive.resetPose(new Pose2d(5, 7, Rotation2d.fromDegrees(0)));
        autonomous().whileTrue(
            drive.wheelRadiusCharacterization()
        );
    }

    private void initLogging() {
        if (RobotMode.get() == RobotMode.REPLAY) {
            setUseTiming(false);
            Logger.setReplaySource(new WPILOGReader(LogFileUtil.findReplayLog()));
        } else {
            var ntPublisher = new NT4Publisher();
            Logger.addDataReceiver(data -> {
                if (DriverStation.isFMSAttached()) return;
                ntPublisher.putTable(data);
            });
        }
        if (!RobotMode.isSim()) {
            Logger.addDataReceiver(new WPILOGWriter());
        }
        Logger.registerURCL(URCL.startExternal());
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("Timestamp", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes commited";
            case 1 -> "There are uncommited changes; replay might be inaccurate";
            default -> "Unknown";
        });
        Logger.start();
    }

    @Override
    public void loopFunc() {
        Tracer.trace("Main", super::loopFunc);
    }

    @Override
    public void robotPeriodic() {
        // TODO Disable setCurrentThreadPriority() if loop times are consistently over 20 ms
        Threads.setCurrentThreadPriority(true, 99);
        SignalBatchRefresher.refreshAll();
        CmdLogger.periodic(true);
        Tracer.trace("Command Scheduler", CommandScheduler.getInstance()::run);
        Logger.recordOutput(
            "LoggedRobot/MemoryUsageMb",
            (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6
        );
        for (var cam: cameras) {
            for (var est: cam.update()) {
                drive.addVisionMeasurement(est);
            }
        }
        Threads.setCurrentThreadPriority(false, 10);
    }
}
