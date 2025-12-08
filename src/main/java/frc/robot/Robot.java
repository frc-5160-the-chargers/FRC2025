package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.commands.CmdLogger;
import frc.chargers.misc.RobotMode;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Tracer;
import frc.robot.components.DriverController;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.ChoreoTrajNames;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

public class Robot extends LoggedRobot {
    private final SwerveDrive drive = new SwerveDrive();
    private final Elevator elevator = new Elevator();
    private final DriverController controller = new DriverController();

    public Robot() {
        // A / (m/s)
        initLogging();
        drive.setDefaultCommand(
            drive.driveCmd(
                controller::forwardOutput,
                controller::strafeOutput,
                controller::rotationOutput,
                true
            )
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator.setDefaultCommand(elevator.idleCmd());
        PortForwarder.add(5800, "photonvision.local", 5800);

        var autoFactory = drive.createAutoFactory();
        autonomous().whileTrue(
            autoFactory.resetOdometry(ChoreoTrajNames.TestPath)
                .andThen(autoFactory.trajectoryCmd(ChoreoTrajNames.TestPath))
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
        Logger.start();

        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes commited";
            case 1 -> "Uncommited changes";
            default -> "Unknown";
        });
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
        if (RobotMode.isSim()) {
            Tracer.trace("MapleSim", SimulatedArena.getInstance()::simulationPeriodic);
        }
        Threads.setCurrentThreadPriority(false, 10);
    }
}
