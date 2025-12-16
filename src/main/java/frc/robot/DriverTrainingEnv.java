package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Tracer;
import frc.robot.components.DriverController;
import frc.robot.subsystems.drive.SwerveConsts;
import frc.robot.subsystems.drive.SwerveDrive;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Seconds;

@SuppressWarnings("ALL")
public class DriverTrainingEnv extends LoggedRobot {
    private final DriverController
        controller1 = new DriverController(0),
        controller2 = new DriverController(1);

    private final SwerveDrive drive, secondaryDrive;

    public DriverTrainingEnv() {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        drive = new SwerveDrive();
        drive.setDefaultCommand(drive.driveCmd(controller1::getSwerveRequest));
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));

        tweakIds();

        secondaryDrive = new SwerveDrive();
        secondaryDrive.setName("MockSecondaryDrivetrain");
        secondaryDrive.setDefaultCommand(
            secondaryDrive.driveCmd(() -> {
                var req = controller2.getSwerveRequest();
                // invert controls, as other driver is on red side
                return req
                    .withVelocityX(req.VelocityX * -1)
                    .withVelocityY(req.VelocityY * -1)
                    .withRotationalRate(req.RotationalRate * -1);
            })
        );
        secondaryDrive.resetPose(new Pose2d(10, 7, Rotation2d.k180deg));

        SimulatedArena.overrideSimulationTimings(Seconds.of(0.002), 1);
        new Notifier(SimulatedArena.getInstance()::simulationPeriodic)
            .startPeriodic(0.002);
    }

    private void tweakIds() {
        // We have to change the Ids of TunerConstants in order to have 2 separate drivetrains
        var availableIds = new ArrayList<Integer>();
        for (int i = 0; i < 30; i++) {
            availableIds.add(i);
        }
        for (var config: SwerveConsts.MODULE_CONSTS_CHOICES) {
            availableIds.removeAll(
                List.of(config.DriveMotorId, config.SteerMotorId, config.EncoderId)
            );
        }
        for (var config: SwerveConsts.MODULE_CONSTS_CHOICES) {
            config.DriveMotorId = availableIds.remove(0);
            config.SteerMotorId = availableIds.remove(0);
            config.EncoderId = availableIds.remove(0);
        }
    }

    @Override
    public void robotPeriodic() {
        Tracer.trace("Command Scheduler", CommandScheduler.getInstance()::run);
        SignalBatchRefresher.refreshAll();
    }
}
