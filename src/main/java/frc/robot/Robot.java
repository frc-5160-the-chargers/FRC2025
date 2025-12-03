package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.commands.CmdLogger;
import frc.chargers.data.RobotMode;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Tracer;
import frc.robot.components.DriverController;
import frc.robot.components.vision.Camera;
import frc.robot.components.vision.VisionConsts;
import frc.robot.constants.BuildConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.teaching.intake.Intake;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

public class Robot extends LoggedRobot {
    private final SwerveDrive drive = new SwerveDrive();
    private final Elevator elevator = new Elevator();
    private final DriverController controller = new DriverController();
    private final Intake intake = new Intake();

    private final List<Camera> cameras = List.of(
//        new Camera(VisionConsts.FL_CONSTS, drive::bestPose),
//        new Camera(VisionConsts.FR_CONSTS, drive::bestPose)
    );

    public Robot() {
        // A / (m/s)
        initLogging();
        drive.setDefaultCommand(
            drive.driveCmd(controller.forwardOutput, controller.strafeOutput, controller.rotationOutput, false)
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        DriverStation.silenceJoystickConnectionWarning(true);
        autonomous().whileTrue(
            drive.runDriveMotors()
        );
        elevator.setDefaultCommand(elevator.idleCmd());
        PortForwarder.add(5800, "photonvision.local", 5800);
    }

    private void initLogging() {
        if (RobotMode.get() == RobotMode.REPLAY) {
            setUseTiming(false);
            Logger.setReplaySource(new WPILOGReader(LogFileUtil.findReplayLog()));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        if (!RobotMode.isSim()) {
            Logger.addDataReceiver(new WPILOGWriter());
        }
        Logger.registerURCL(URCL.startExternal());
        Logger.start();

        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", String.valueOf(BuildConstants.DIRTY));
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    }

    @Override
    public void loopFunc() {
        Tracer.trace("Main", super::loopFunc);
    }

    private void cameraPeriodic() {
        for (var cam: cameras) {
            var estimates = cam.update();
            for (var estimate: estimates) {
                drive.addVisionData(estimate);
            }
        }
    }

    @Override
    public void robotPeriodic() {
        SignalBatchRefresher.refreshAll();
        CmdLogger.periodic(true);
        Tracer.trace("Command Scheduler", CommandScheduler.getInstance()::run);
        Tracer.trace("Camera Periodic", this::cameraPeriodic);
        if (RobotMode.isSim()) {
            Tracer.trace("MapleSim", SimulatedArena.getInstance()::simulationPeriodic);
        }
    }
}
