package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.data.RobotMode;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Tracer;
import frc.robot.components.DriverController;
import frc.robot.components.vision.Camera;
import frc.robot.components.vision.VisionConsts;
import frc.robot.subsystems.drive.SwerveDrive;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.List;

public class Robot extends LoggedRobot {
    private final SwerveDrive drive = new SwerveDrive();
    private final DriverController controller = new DriverController();

    private final List<Camera> cameras = List.of(
        new Camera(VisionConsts.FL_CONSTS, drive::bestPose),
        new Camera(VisionConsts.FR_CONSTS, drive::bestPose)
    );

    public Robot() {
        if (RobotMode.get() == RobotMode.REPLAY) {
            setUseTiming(false);
            Logger.setReplaySource(new WPILOGReader(LogFileUtil.findReplayLog()));
            Tracer.disableTracingForCurrentThread();
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.start();
        drive.setDefaultCommand(
            drive.driveCmd(controller.forwardOutput, controller.strafeOutput, controller.rotationOutput, false)
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void loopFunc() {
        Tracer.trace("Main", super::loopFunc);
    }

    @Override
    public void robotPeriodic() {
        Tracer.trace("Signal Refresh", SignalBatchRefresher::refreshAll);
        Tracer.trace("Cmd Scheduler", CommandScheduler.getInstance()::run);
        if (RobotMode.isSim()) {
            Tracer.trace("MapleSim", SimulatedArena.getInstance()::simulationPeriodic);
        }
        for (var cam: cameras) {
            var estimates = cam.update();
            for (var estimate: estimates) {
                drive.addVisionData(estimate);
            }
        }
    }
}
